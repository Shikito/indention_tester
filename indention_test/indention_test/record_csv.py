import os
from pathlib import Path
from datetime import datetime as dt

import csv
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32

class RecordCSV(Node):
    def __init__(self):
        super().__init__('record_csv')

        # Set Bag Dir
        tdatetime = dt.now()
        tstr = tdatetime.strftime('%Y%m%d%H%M%S')
        self.csv_dir_name = f'/home/toshi/dev_ws/csv/indention_test/{tstr}'
        Path.mkdir(self.csv_dir_name, exist_ok=True)
        self.csv_column_count = 0
        self.csv_max_column = 1000
        self.csv_file_number = 0

        self.bend_sensor_value = None
        self.target_pressure_value = None
        self.current_pressure_value = None
        self.ttac3_state = None

        self.sub_bend_sensor = self.create_subscription(
            Int32,
            '/bend_sensor/int',
            self.callback_bend_sensor,
            10
        )

        self.sub_ttac3_state = self.create_subscription(
            Int32MultiArray,
            '/ttac3_state',
            self.callback_ttac3_state,
            10
        )

        self.sub_target_pressure = self.create_subscription(
            Float32,
            '/terminal/int/target_pressure',
            self.callback_target_pressure,
            10
        )

        self.sub_current_pressure = self.create_subscription(
            Float32,
            '/terminal/int/current_pressure',
            self.callback_current_pressure,
            10
        )

        period_sec = 0.01
        self.timer = self.create_timer(period_sec, self.callback_write_csv)

    def callback_bend_sensor(self, msg):
        self.bend_sensor_value = msg.data

    def callback_target_pressure(self, msg):
        self.target_pressure_value = msg.data

    def callback_current_pressure(self, msg):
        self.current_pressure_value = msg.data

    def callback_ttac3_state(self, msg):
        self.ttac3_state = msg.data

    def callback_write_csv(self):
        csv_file_number = self.csv_column_count//self.csv_max_column
        file_name = f'sample_{csv_file_number}.csv'
        with open(f'{self.csv_dir_name}/{file_name}', 'a') as f:
            writer = csv.writer(f)

            is_new_file = self.csv_file_number != csv_file_number
            if is_new_file: 
                writer.writerow([
                'bend_sensor_value',
                'target_pressure_value',
                'current_pressure_value',
                'ttac3_state_0',
                'ttac3_state_1',
                'ttac3_state_2',
                ])
                self.csv_file_number = csv_file_number
            
            writer.writerow([
                self.bend_sensor_value,
                self.target_pressure_value,
                self.current_pressure_value,
                self.ttac3_state[0],
                self.ttac3_state[1],
                self.ttac3_state[2],
            ])
            self.csv_column_count += 1



def main():
    
    rclpy.init()
    
    recoder = RecordCSV()

    rclpy.spin(recoder)

    rclpy.shutdown()

if __name__=='__main__':
    main()