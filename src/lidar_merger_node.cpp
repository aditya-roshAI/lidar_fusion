#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>

#include <pcl_ros/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <map>
#include <vector>
#include <string>
#include <mutex>
#include <memory>

struct CloudData {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    rclcpp::Time timestamp;
    std::string frame;

    CloudData() : cloud(new pcl::PointCloud<pcl::PointXYZ>), timestamp(0), frame("") {}
    CloudData(pcl::PointCloud<pcl::PointXYZ>::Ptr c, rclcpp::Time t, std::string f)
        : cloud(c), timestamp(t), frame(f) {}
};

class DynamicLidarMerger : public rclcpp::Node
{
public:
    DynamicLidarMerger()
    : Node("dynamic_lidar_merger"),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_)
    {
        // Declare and get parameters
        this->declare_parameter<std::string>("target_frame", "base_link");
        this->declare_parameter<double>("max_time_diff", 1.0);
        this->declare_parameter<double>("merge_timeout", 0.5);
        this->declare_parameter<int>("min_lidars_for_merge", 2);
        this->declare_parameter<std::vector<std::string>>(
            "lidar_topics", std::vector<std::string>{
                "/ouster/points", "/ouster_right/points"
            });

        target_frame_ = this->get_parameter("target_frame").as_string();
        max_time_diff_ = this->get_parameter("max_time_diff").as_double();
        merge_timeout_ = this->get_parameter("merge_timeout").as_double();
        min_lidars_for_merge_ = this->get_parameter("min_lidars_for_merge").as_int();
        lidar_topics_ = this->get_parameter("lidar_topics").as_string_array();

        RCLCPP_INFO(this->get_logger(), "Configuring merger for %zu LiDAR topics", lidar_topics_.size());
        RCLCPP_INFO(this->get_logger(), "Target frame: %s", target_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "Max time difference: %.3f seconds", max_time_diff_);
        RCLCPP_INFO(this->get_logger(), "Merge timeout: %.3f seconds", merge_timeout_);
        RCLCPP_INFO(this->get_logger(), "Minimum LiDARs for merge: %d", min_lidars_for_merge_);

        rclcpp::QoS best_effort_qos_profile(1); // depth = 1
        best_effort_qos_profile
            .reliability(rclcpp::ReliabilityPolicy::BestEffort)
            .history(rclcpp::HistoryPolicy::KeepLast);

        // Dynamic subscribers
        for (const auto& topic : lidar_topics_) {
            auto sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                topic, best_effort_qos_profile,
                [this, topic](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                    this->cloudCallback(msg, topic);
                }
            );
            subscribers_.push_back(sub);
        }

        // Publisher
        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/fused_cloud", 1);

        // Timer (50ms)
        merge_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&DynamicLidarMerger::checkMergeTimeout, this));

        expected_lidar_count_ = lidar_topics_.size();
        subscribers_count_ = lidar_topics_.size();
        last_update_time_ = this->now();
    }

private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg, const std::string& topic_name)
    {
        rclcpp::Time current_time = cloud_msg->header.stamp;

        // Convert to PCL
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*cloud_msg, *pcl_cloud);

        // Transform to target frame
        sensor_msgs::msg::PointCloud2 transformed_cloud;
        try {
            if (!tf_buffer_.canTransform(target_frame_, cloud_msg->header.frame_id,
                                         cloud_msg->header.stamp, rclcpp::Duration(0, 200 * 1e6))) {
                RCLCPP_WARN(this->get_logger(), "Cannot transform from %s to %s",
                            cloud_msg->header.frame_id.c_str(), target_frame_.c_str());
                return;
            }
            pcl_ros::transformPointCloud(target_frame_, *cloud_msg, transformed_cloud, tf_buffer_);
        }
        catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Transform failed for %s: %s", topic_name.c_str(), ex.what());
            return;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_pcl(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(transformed_cloud, *transformed_pcl);

        // Store in buffer with timestamp
        {
            std::lock_guard<std::mutex> lock(buffer_mutex_);
            cloud_buffer_[topic_name] = CloudData(transformed_pcl, current_time, cloud_msg->header.frame_id);
            last_update_time_ = this->now();
        }

        // Try to merge if we have enough clouds
        attemptMerge();
    }

    void attemptMerge()
    {
        int rejected_clouds = 0;
        std::lock_guard<std::mutex> lock(buffer_mutex_);

        if (cloud_buffer_.size() < min_lidars_for_merge_) {
            return;
        }

        // Find reference timestamp (most recent)
        rclcpp::Time reference_time(0, 0, this->get_clock()->get_clock_type());
        for (const auto& pair : cloud_buffer_) {
            if (pair.second.timestamp > reference_time) {
                reference_time = pair.second.timestamp;
            }
        }

        // Check time synchronization and collect valid clouds
        std::vector<CloudData> valid_clouds;
        std::vector<std::string> rejected_topics;

        for (const auto& pair : cloud_buffer_) {
            double time_diff = std::abs((pair.second.timestamp - reference_time).seconds());
            if (time_diff <= max_time_diff_) {
                valid_clouds.push_back(pair.second);
            } else {
                rejected_topics.push_back(pair.first);
                RCLCPP_WARN(this->get_logger(), "Rejected cloud from %s due to time difference: %.3f seconds",
                            pair.first.c_str(), time_diff);
                rejected_clouds++;
            }
        }

        // Remove rejected clouds from buffer
        for (const std::string& topic : rejected_topics) {
            cloud_buffer_.erase(topic);
        }

        // Merge if we have enough valid clouds
        if (valid_clouds.size() >= min_lidars_for_merge_) {
            if (subscribers_count_ - rejected_clouds == (int)valid_clouds.size()) {
                mergeAndPublish(valid_clouds, reference_time);
                cloud_buffer_.clear();
            }
        }
    }

    void mergeAndPublish(const std::vector<CloudData>& clouds, rclcpp::Time reference_time)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        size_t total_points = 0;
        for (const auto& cloud_data : clouds) {
            *merged_cloud += *(cloud_data.cloud);
            total_points += cloud_data.cloud->points.size();
        }

        // Convert back to ROS2 message
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*merged_cloud, output);
        output.header.frame_id = target_frame_;
        output.header.stamp = reference_time;

        pub_->publish(output);

        RCLCPP_INFO(
            this->get_logger(),
            "Merged --- %zu --- clouds with %zu total points at time %.3f",
            clouds.size(), total_points, reference_time.seconds()
        );
    }

    void checkMergeTimeout()
    {
        std::lock_guard<std::mutex> lock(buffer_mutex_);

        if (cloud_buffer_.empty()) return;

        rclcpp::Time current_time = this->now();
        double time_since_update = (current_time - last_update_time_).seconds();

        // If timeout exceeded, try partial merge or clear old data
        if (time_since_update > merge_timeout_) {
            if (cloud_buffer_.size() >= min_lidars_for_merge_) {
                // Find reference time among clouds
                rclcpp::Time reference_time(0, 0, this->get_clock()->get_clock_type());
                for (const auto& pair : cloud_buffer_) {
                    if (pair.second.timestamp > reference_time)
                        reference_time = pair.second.timestamp;
                }
                std::vector<CloudData> available_clouds;
                for (const auto& pair : cloud_buffer_)
                    available_clouds.push_back(pair.second);
                mergeAndPublish(available_clouds, reference_time);
            } else {
                RCLCPP_WARN(this->get_logger(), "Clearing stale cloud buffer (timeout reached, insufficient clouds)");
            }
            cloud_buffer_.clear();
            last_update_time_ = current_time;
        }
    }

    // Class members
    std::vector<std::string> lidar_topics_;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> subscribers_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr merge_timer_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Parameters
    std::string target_frame_;
    double max_time_diff_;
    double merge_timeout_;
    int min_lidars_for_merge_;
    size_t expected_lidar_count_;
    int subscribers_count_;

    std::map<std::string, CloudData> cloud_buffer_;
    std::mutex buffer_mutex_;
    rclcpp::Time last_update_time_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DynamicLidarMerger>();
    RCLCPP_INFO(node->get_logger(), "Dynamic LiDAR Merger started successfully");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}