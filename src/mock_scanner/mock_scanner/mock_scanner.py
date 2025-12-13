import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import random

BARCODES = ["TCF150", "KPT568)", "PLLEO93", "BMVALDE912"]

class MockScanner(Node):
    def __init__(self):
        super().__init__("mock_scanner")
        self.pub = self.create_publisher(String, "barcode", 10)
        self.sub = self.create_subscription(Bool, "scanner_trigger", self.on_trigger, 10)

        self.scanning = False
        self.timer = self.create_timer(0.5, self.publish_barcode)
        self.get_logger().info("MockScanner started")

    def on_trigger(self, msg):
        self.scanning = msg.data
        self.get_logger().info(f"Scanner trigger: {self.scanning}")

    def publish_barcode(self):
        if self.scanning:
            msg = String()
            msg.data = random.choice(BARCODES)
            self.pub.publish(msg)
            self.get_logger().info(f"Publishing barcode: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = MockScanner()
    rclpy.spin(node)
