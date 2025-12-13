import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class MockTableSensor(Node):
    def __init__(self):
        super().__init__("mock_table_sensor")  # Name this node

        # Publisher for the table occupancy status
        # True = table is clear, False = table occupied
        self.pub = self.create_publisher(Bool, "table_is_clear", 10)

        # Timer that periodically calls self.publish every 1.5 seconds
        # Simulates a sensor checking the table status regularly
        self.timer = self.create_timer(1.5, self.publish)

    def publish(self):
        """
        Timer callback that publishes the table status.
        Here we are always publishing True to simulate a clear table.
        """
        msg = Bool()
        msg.data = True  # The table is considered empty
        self.pub.publish(msg)  # Send the message

def main():
    rclpy.init()
    rclpy.spin(MockTableSensor())  # Keep the node running
