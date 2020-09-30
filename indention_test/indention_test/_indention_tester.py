import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import Float32
from ttac3_actions.action import TTAC3
# from indention_test_utils.ttac3_client_node import TTAC3ClientNode
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
        
        # Publish Target Air Pressure
        self.pub_target_air_pressure = self.create_publisher(Float32,'target_air_pressure', 10)
        pub_timer = 0.5 # seconds
        self.target_air_pressure = 150.0 # target_air_pressure
        self.timer = self.create_timer(pub_timer, self.pub_target_air_pressure_callback)

        # Subscribe Current Air Pressure
        self.sub_current_air_pressure = self.create_subscription(
            Float32,
            'current_air_pressure',
            self.current_air_pressure_listener_callback,
            10
        )

        # Subscribe Bend Sensor
        self.sub_bend_sensor = self.create_subscription(
            Float32,
            'bend_sensor',
            self.bend_sensor_listener_callback,
            10
        )
    
    def bend_sensor_listener_callback(self, msg):
        # self.get_logger().info(f'Bend Sensor Value is {msg.data}')
        self.bend_sensor = msg.data

    def current_air_pressure_listener_callback(self, msg):
        # self.get_logger().info(f'Current Air Pressure is {msg.data}')
        self.current_air_pressure = msg.data

    def pub_target_air_pressure_callback(self):
        msg = Float32()
        msg.data = self.target_air_pressure
        self.pub_target_air_pressure.publish(msg)
        # self.get_logger().info(f'Publishing: Target Air Pressure == {msg.data}')

    def update(self):
        home_position = [49, 137, 1]
        indent_position = [49, 137, 20]
        self.get_logger().info('on update')
        self.ttac3_send_goal_sync(home_position)
        # import ipdb; ipdb.set_trace()
        self.get_logger().info('on update2')
        # self.ttac3_send_goal_sync(indent_position)
        # self.get_logger().info('on update3')
        # self.ttac3_send_goal_sync(home_position)
    
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
        # rclpy.shutdown()

def main(args=sys.argv):
    rclpy.init()
    indention_tester = IndentionTester()
    indention_tester.main_thread.start()
    rclpy.spin(indention_tester)


if __name__ == '__main__':
    main()
