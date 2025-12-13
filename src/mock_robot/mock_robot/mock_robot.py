import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class MockRobot(Node):
    def __init__(self):
        super().__init__('mock_robot')
        self.ready_sub = self.create_subscription(
            Bool, 'ready_for_next_item', self.on_ready, 10)
        self.item_pub = self.create_publisher(Bool, 'item_placed', 10)

        self.ready = True
        self.timer = self.create_timer(1.0, self.try_place)
        self.get_logger().info("MockRobot started")

    def on_ready(self, msg):
        self.ready = msg.data

    def try_place(self):
        if not self.ready:
            return
        msg = Bool()
        msg.data = True
        self.item_pub.publish(msg)
        self.ready = False
        self.get_logger().info("Placed item")

def main():
    rclpy.init()
    rclpy.spin(MockRobot())
