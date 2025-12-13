import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class MockRobot(Node):
    def __init__(self):
        super().__init__('mock_robot')  # Name this node

        # Subscriber to know when the ScanTableManager is ready for the next item
        self.ready_sub = self.create_subscription(
            Bool, 'ready_for_next_item', self.on_ready, 10)

        # Publisher to indicate that an item has been placed on the scan table
        self.item_pub = self.create_publisher(Bool, 'item_placed', 10)

        # Flag indicating if the robot can place a new item
        self.ready = True

        # Timer to attempt placing an item every 1 second
        # Only places if ready is True
        self.timer = self.create_timer(1.0, self.try_place)

        self.get_logger().info("MockRobot started")

    def on_ready(self, msg):
        """
        Callback for when the ScanTableManager signals readiness.
        Sets self.ready to True if it receives a True message.
        """
        self.ready = msg.data

    def try_place(self):
        """
        Timer callback called every 1 second.
        Publishes an item_placed message if the robot is ready.
        """
        if not self.ready:
            return  # Do nothing if not ready

        msg = Bool()
        msg.data = True
        self.item_pub.publish(msg)  # Publish that an item is placed
        self.ready = False  # Wait until ScanTableManager signals readiness again
        self.get_logger().info("Placed item")

def main():
    rclpy.init()
    rclpy.spin(MockRobot())  # Keep the node running
