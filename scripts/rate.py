import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import time

class TopicRateMonitor(Node):
    def __init__(self):
        super().__init__('topic_rate_monitor_dual')
        self.last_msg_time = {
            "/ouster1/ouster1/points": None,
            "/ouster2/ouster2/points": None
        }
        self.msg_count = {
            "/ouster1/ouster1/points": 0,
            "/ouster2/ouster2/points": 0
        }

        self.rate_check_window = 10.0  # seconds
        self.drop_threshold = 1.0      # seconds of silence before warning

        self.sub1 = self.create_subscription(
            PointCloud2, "/ouster1/ouster1/points", self.callback_ouster1, 10)
        self.sub2 = self.create_subscription(
            PointCloud2, "/ouster2/ouster2/points", self.callback_ouster2, 10
        )

        # ROS2 timer - fires every 1 second
        self.create_timer(1.0, self.timer_callback)

    def callback_ouster1(self, msg):
        self._update_topic("/ouster1/ouster1/points")

    def callback_ouster2(self, msg):
        self._update_topic("/ouster2/ouster2/points")

    def _update_topic(self, topic_name):
        now = time.time()
        if self.last_msg_time[topic_name] is not None:
            interval = now - self.last_msg_time[topic_name]
            # self.get_logger().info(f"{topic_name} Interval: {interval:.3f} s (~{1.0/interval:.2f} Hz)")
        self.last_msg_time[topic_name] = now
        self.msg_count[topic_name] += 1

    def timer_callback(self):
        for topic_name in self.last_msg_time:
            last_time = self.last_msg_time[topic_name]
            if last_time is None:
                self.get_logger().warn(f"No new messages received yet on {topic_name}.")
            else:
                silence = time.time() - last_time
                if silence > self.drop_threshold:
                    self.get_logger().warn(
                        f"No messages received on {topic_name} in the last {silence:.1f} seconds! Possible drop.")
                else:
                    self.get_logger().debug(
                        f"{topic_name} last message {silence:.2f} s ago.")

def main(args=None):
    rclpy.init(args=args)
    node = TopicRateMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()