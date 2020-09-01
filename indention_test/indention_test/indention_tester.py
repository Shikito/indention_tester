import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from ttac3_actions.action import TTAC3
from indention_test_utils.ttac3_client_node import TTAC3ClientNode

class IndentionTester(TTAC3ClientNode):
    def __init__(self):
        super().__init__('indention_tester')
        


def main(args=None):
    rclpy.init()
    indention_tester = IndentionTester()
    indention_tester.ttac3_send_goal([int(n) for n in args[1:]])
    rclpy.spin(indention_tester)


if __name__ == '__main__':
    main()
