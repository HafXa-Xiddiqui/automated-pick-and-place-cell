import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

class MockTableSensor(Node):
    def __init__(self):
        super().__init__("mock_table_sensor")
        self.pub = self.create_publisher(Bool, "/table/occupied", 10)
        self.sub_pusher = self.create_subscription(String, "/pusher/status", self.pusher_cb, 10)
        self.sub_robot  = self.create_subscription(Bool, "/robot/item_placed", self.robot_cb, 10)

    def robot_cb(self, msg):
        out = Bool()
        out.data = True
        self.pub.publish(out)

    def pusher_cb(self, msg):
        if msg.data == "finished":
            out = Bool()
            out.data = False
            self.pub.publish(out)

def main():
    rclpy.init()
    node = MockTableSensor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
