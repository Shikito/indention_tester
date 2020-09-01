import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from ttac3_actions.action import TTAC3

class TTAC3ClientNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self._ttac3_action_client = ActionClient(
            self, TTAC3, 'ttac3'
        )

    def ttac3_send_goal(self, xyz_goal):
        goal_msg = TTAC3.Goal()
        goal_msg.xyz_goal = xyz_goal
        self._ttac3_action_client.wait_for_server()
        self._send_goal_future = self._ttac3_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.ttac3_feedback_callback
        )
        self._send_goal_future.add_done_callback(self.ttac3_goal_response_callback)
        

    def ttac3_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.current_state))

    def ttac3_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected: (')
            return
        
        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.ttac3_get_result_callback)
    
    def ttac3_get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.is_success))