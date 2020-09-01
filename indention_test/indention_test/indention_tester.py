import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from ttac3_actions.action import TTAC3
from indention_test_utils.ttac3_client_node import TTAC3ClientNode
from yi2016_utils.node_utils import create_thread

class IndentionTester(Node):
    def __init__(self):
        super().__init__('indention_tester')
        self._ttac3_action_client = ActionClient(
            self, TTAC3, 'ttac3'
        )

        # Main Loop (by Thread)
        update_period = 0.5
        self.main_thread = create_thread(update_period, self.update)
    
    def update(self):
        self.get_logger().info('on update')
        self.ttac3_send_goal_sync([100, 100, 1])
    
    def ttac3_send_goal_sync(self, xyz_goal):
        goal_msg = TTAC3.Goal()
        goal_msg.xyz_goal = xyz_goal
        self._ttac3_action_client.wait_for_server()
        self.get_logger().info(f'Request was Accepted by ttac3_server')
        result = self._ttac3_action_client.send_goal(
            goal_msg,
            feedback_callback=None
        )
        self.get_logger().info(f'Result is {result.result.is_success}')
        rclpy.shutdown()

def main(args=sys.argv):
    rclpy.init()
    indention_tester = IndentionTester()
    indention_tester.main_thread.start()
    rclpy.spin(indention_tester)


if __name__ == '__main__':
    main()
