import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

class MockPusher(Node):
    def __init__(self):
        super().__init__("mock_pusher")  # Name this node

        # Subscriber to listen for pusher commands (POCKET or REJECT)
        self.sub = self.create_subscription(
            String, "pusher_cmd", self.on_cmd, 10)

        # Publisher to indicate that the pusher has finished its movement
        self.done_pub = self.create_publisher(Bool, "pusher_done", 10)

    def on_cmd(self, msg):
        """
        Callback executed when a pusher command is received.
        Logs the command and immediately publishes pusher_done.
        """
        # Log the action being taken
        self.get_logger().info(f"Pushing to {msg.data}")

        # Publish that the push action is complete
        done = Bool()
        done.data = True
        self.done_pub.publish(done)

def main():
    rclpy.init()
    rclpy.spin(MockPusher())  # Keep the node running
