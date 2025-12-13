import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

class MockPusher(Node):
    def __init__(self):
        super().__init__("mock_pusher")
        self.sub = self.create_subscription(String, "pusher_cmd", self.on_cmd, 10)
        self.done_pub = self.create_publisher(Bool, "pusher_done", 10)

    def on_cmd(self, msg):
        self.get_logger().info(f"Pushing to {msg.data}")
        done = Bool()
        done.data = True
        self.done_pub.publish(done)

def main():
    rclpy.init()
    rclpy.spin(MockPusher())
