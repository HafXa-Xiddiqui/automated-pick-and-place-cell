import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class MockTableSensor(Node):
    def __init__(self):
        super().__init__("mock_table_sensor")
        self.pub = self.create_publisher(Bool, "table_is_clear", 10)
        self.timer = self.create_timer(1.5, self.publish)

    def publish(self):
        msg = Bool()
        msg.data = True
        self.pub.publish(msg)

def main():
    rclpy.init()
    rclpy.spin(MockTableSensor())
