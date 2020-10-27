import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import Float32
from std_msgs.msg import Int32MultiArray
from ttac3_interfaces.srv import TTAC3
# from indention_test_utils.ttac3_client_node import TTAC3ClientNode
from yi2016_utils.node_utils import create_thread
from ros2serial_interfaces.srv import ROS2SerialSrv

class IndentionTester(Node):
    def __init__(self):
        super().__init__('indention_tester')

        self.cli_ttac3 = self.create_client(
            TTAC3, 'ttac3')
        while not self.cli_ttac3.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('service not available, waiting again...')
        self.req_ttac3 = TTAC3.Request()

        # Set Target Air Pressure
        self.cli_target_air_pressure = self.create_client(
            ROS2SerialSrv, '/terminal/write')
        while not self.cli_target_air_pressure.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('service not available, waiting again...')
        self.req_target_air_pressure = ROS2SerialSrv.Request()
        
        # Main Loop (by Thread)
        update_period = 0.5
        self.count = 0
        self.main_thread = create_thread(update_period, self.update)
        
        # Publish Target Air Pressure
        self.pub_target_air_pressure = self.create_publisher(Float32,'target_air_pressure', 10)
        pub_timer = 0.5 # seconds
        self.target_air_pressure = 100.0 # target_air_pressure
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
        self.bend_sensor = msg.data

    def current_air_pressure_listener_callback(self, msg):
        self.current_air_pressure = msg.data

    def pub_target_air_pressure_callback(self):
        msg = Float32()
        msg.data = self.target_air_pressure
        self.pub_target_air_pressure.publish(msg)

    def update(self):
        home_position = [130, 83, 80]
        indent_position = [130, 83, 85]
        target_air_pressure_list = [100, 125, 150, 175, 200]

        trial_num = 3
        for i in range(trial_num):
            self.get_logger().info(f'***{i} / {trial_num} Trial ***')
            for target_air_pressure in target_air_pressure_list:

                # Set target_air_pressure
                self.get_logger().info(f'Send target_air_pressure : {target_air_pressure}')
                self.req_target_air_pressure.data = str(target_air_pressure)
                response = self.request_service_sync(
                    self.cli_target_air_pressure,
                    self.req_target_air_pressure
                )
                self.get_logger().info(f'Response : {response.success}')
                
                # Move To Home Position
                self.req_ttac3.xyz_goal = home_position
                self.get_logger().info(f'Send xyz_goal : {self.req_ttac3.xyz_goal}')
                response = self.request_service_sync(
                    self.cli_ttac3, self.req_ttac3)
                self.get_logger().info(f'Response : {response.is_success}')
                time.sleep(1)

                # Move To Indent Position
                self.req_ttac3.xyz_goal = indent_position
                self.get_logger().info(f'Send xyz_goal : {self.req_ttac3.xyz_goal}')
                response = self.request_service_sync(
                    self.cli_ttac3, self.req_ttac3)
                self.get_logger().info(f'Response : {response.is_success}')
                time.sleep(1)

        while True:
            self.get_logger().info('Complete Task : Please Press Ctrl-C to End')
            time.sleep(1)

    def request_service_sync(self, client, req):
        future = client.call_async(req)
        self.get_logger().info('waiting for response')
        while not future.done():
            time.sleep(0.01)
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().info(
                'Service call failed %r' % (e,))

        return response


def main(args=sys.argv):
    rclpy.init()
    indention_tester = IndentionTester()
    indention_tester.main_thread.start()
    rclpy.spin(indention_tester)

if __name__ == '__main__':
    main()
