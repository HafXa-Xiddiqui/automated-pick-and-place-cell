import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
import random

class MockScanner(Node):
    def __init__(self):
        super().__init__("mock_scanner")
        self.sub = self.create_subscription(Bool, "/scanner/trigger", self.trigger_cb, 10)
        self.pub = self.create_publisher(String, "/scanner/barcode", 10)

    def trigger_cb(self, msg):
        if msg.data:
            if random.random() < 0.7:
                for _ in range(random.randint(1, 3)):
                    s = String()
                    s.data = f"CODE_{random.randint(1000,9999)}"
                    self.pub.publish(s)
            else:
                pass

def main():
    rclpy.init()
    node = MockScanner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
