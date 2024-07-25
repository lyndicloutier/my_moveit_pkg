# pylint: disable=attribute-defined-outside-init, missing-module-docstring, missing-class-docstring, missing-final-newline, missing-function-docstring, trailing-whitespace, line-too-long

import rclpy
from rclpy.node import Node
from ur5_arm_msgs.srv import UR5ServiceMessage

class Client(Node):
    def __init__(self):
        super().__init__("publisher_testing")
        self.client_ = self.create_client(UR5ServiceMessage, 'posestamped')
        while not self.client_.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info("service not available, waiting again...")
        self.request = UR5ServiceMessage.Request()

    def send_request(self):
        self.request.position.pose.position.x = 0.0
        self.request.position.pose.position.y = 1.0
        self.request.position.pose.position.z = 2.0
        self.request.position.header.frame_id = "world"
        self.future = self.client_.call_async(self.request)
        self.get_logger().info("test\n")
        rclpy.spin_until_future_complete(self, self.future)
        self.get_logger().info("Publishing position\n")
        # self.get_logger().info("Publishing: position = (" + str(self.request.position.position.x) + ", " + str(self.request.position.position.y) + ", " + str(self.request.position.position.z) + ") and frame_id = " + self.request.position.header.frame_id)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    client = Client()
    client.send_request()
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()