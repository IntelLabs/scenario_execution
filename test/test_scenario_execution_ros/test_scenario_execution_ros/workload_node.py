import sys
import os
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import String


class TestNode(Node):

    def __init__(self):
        super().__init__('test_node')
        self.declare_parameter('test_path', rclpy.Parameter.Type.STRING)
        self.declare_parameter('test_param', rclpy.Parameter.Type.STRING)
        self.test_path = self.get_parameter_or('test_path', '.').value
        self.test_param = self.get_parameter_or('test_param', '').value
        self.get_logger().info(f"Writing to '{self.test_path}'...")
        if not os.path.exists(self.test_path):
            os.makedirs(self.test_path)
        with open(os.path.join(self.test_path, 'test_started'), 'w') as f:
            f.write(self.test_param)
        self.timer = self.create_timer(5, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info('Timeout')
        raise SystemExit()

def main(args=None):
    rclpy.init(args=args)

    result = True
    try:
        node = TestNode()

        rclpy.spin(node)
    except SystemExit:
        pass
    except BaseException:
        result = False
    
    if result:
        print("Success!")
        with open(os.path.join(node.test_path, 'test_success'), 'w') as f:
            f.write(node.test_param)
    else:
        print("Aborted!")
        with open(os.path.join(node.test_path, 'test_aborted'), 'w') as f:
            f.write(node.test_param)


if __name__ == '__main__':
    main()