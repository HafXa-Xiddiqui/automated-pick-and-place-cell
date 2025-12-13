import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import random

# List of example barcodes that the scanner can detect
BARCODES = ["TCF150", "KPT568)", "PLLEO93", "BMVALDE912"]

class MockScanner(Node):
    def __init__(self):
        super().__init__("mock_scanner")  # Give this node a name

        # Publisher to send barcode data to the ScanTableManager
        self.pub = self.create_publisher(String, "barcode", 10)

        # Subscription to the scanner trigger topic
        # In triggered mode, the ScanTableManager will tell this node when to scan
        self.sub = self.create_subscription(
            Bool, "scanner_trigger", self.on_trigger, 10
        )

        # Flag to indicate whether the scanner is currently active
        self.scanning = False

        # Timer that runs every 0.5 seconds to attempt to publish a barcode
        # Will only actually publish if self.scanning is True
        self.timer = self.create_timer(0.5, self.publish_barcode)

        self.get_logger().info("MockScanner started")

    def on_trigger(self, msg):
        """
        Callback for when a scanner trigger message is received.

        Args:
            msg (Bool): True = start scanning, False = stop scanning
        """
        self.scanning = msg.data  # Update scanning state
        self.get_logger().info(f"Scanner trigger: {self.scanning}")

    def publish_barcode(self):
        """
        Timer callback called every 0.5 seconds.

        Publishes a random barcode only if scanning is enabled.
        This simulates the scanner capturing an item on the table.
        """
        if self.scanning:
            msg = String()
            msg.data = random.choice(BARCODES)  # pick a barcode randomly
            self.pub.publish(msg)  # send it out
            self.get_logger().info(f"Publishing barcode: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = MockScanner()  # create the scanner node
    rclpy.spin(node)      # keep it running until the program is killed
