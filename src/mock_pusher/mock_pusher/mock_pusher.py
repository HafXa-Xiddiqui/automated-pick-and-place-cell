import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import threading

class MockPusher(Node):
    def __init__(self):
        super().__init__('mock_pusher')
        self.sub = self.create_subscription(String, '/pusher/cmd', self.cmd_cb, 10)
        self.pub = self.create_publisher(String, '/pusher/status', 10)

    def cmd_cb(self, msg):
        def do_push():
            time.sleep(1.0)
            out = String()
            out.data = "finished"
            self.pub.publish(out)

        threading.Thread(target=do_push).start()

def main():
    rclpy.init()
    node = MockPusher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
