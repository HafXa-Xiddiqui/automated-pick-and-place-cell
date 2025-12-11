import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class MockRobot(Node):
    def __init__(self):
        super().__init__('mock_robot')
        self.sub = self.create_subscription(Bool, '/ready_for_next_item', self.ready_cb, 10)
        self.pub = self.create_publisher(Bool, '/robot/item_placed', 10)

    def ready_cb(self, msg):
        if msg.data:
            self.get_logger().info("Robot placing item.")
            out = Bool()
            out.data = True
            self.pub.publish(out)

def main():
    rclpy.init()
    node = MockRobot()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
