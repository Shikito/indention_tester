import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import Float32
from ttac3_interfaces.srv import TTAC3
# from indention_test_utils.ttac3_client_node import TTAC3ClientNode
from yi2016_utils.node_utils import create_thread

class IndentionTester(Node):
    def __init__(self):
        super().__init__('indention_tester')

        self.cli_ttac3 = self.create_client(
            TTAC3, 'ttac3')
        self.req = TTAC3.Request()
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
        self.get_logger().info('on update')
        if self.count % 2 == 0:
            self.req.xyz_goal = home_position
        else:
            self.req.xyz_goal = indent_position
        self.get_logger().info(f'Send xyz_goal : {self.req.xyz_goal}')
        response = self.request_service_sync(
            self.cli_ttac3, self.req)
        self.get_logger().info(f'Response : {response.is_success}')
        self.count += 1

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
