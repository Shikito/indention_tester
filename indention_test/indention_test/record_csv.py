import os
import sys
import argparse
from time import time
from pathlib import Path
from datetime import datetime as dt

import csv
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32

class RecordCSV(Node):
    def __init__(self, object_width, object_radious):
        super().__init__('record_csv')

        # Set Bag Dir
        self.csv_dir_name = f'/home/toshi/dev_ws/csv/indention_test/'
        Path(self.csv_dir_name).mkdir(parents=True)
        self.start_time = time()
        self.csv_column_count = 0
        self.csv_max_column = 1000
        self.csv_file_number = -1
        self.object_width = object_width
        self.object_radious = object_radious

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
        written_data = [
                self.bend_sensor_value,
                self.target_pressure_value,
                self.current_pressure_value,
                self.ttac3_state
            ]
        if None in written_data:
            return 
        
        csv_file_number = self.csv_column_count//self.csv_max_column
        csv_file_name = f'width_{self.object_width}_radious_{self.object_radious}_number{csv_file_number}.csv'
        
        with open(f'{self.csv_dir_name}/{csv_file_name}', 'a') as f:
            writer = csv.writer(f)

            is_new_file = self.csv_file_number != csv_file_number
            if is_new_file: 
                writer.writerow([
                'time_stamp',
                'bend_sensor_value',
                'target_pressure_value',
                'current_pressure_value',
                'ttac3_state_0',
                'ttac3_state_1',
                'ttac3_state_2',
                ])
                self.csv_file_number = csv_file_number
            
            writer.writerow([
                time() - self.start_time,
                self.bend_sensor_value,
                self.target_pressure_value,
                self.current_pressure_value,
                self.ttac3_state[0],
                self.ttac3_state[1],
                self.ttac3_state[2],
            ])
            self.csv_column_count += 1



def main(argv=sys.argv):

    parser = argparse.ArgumentParser(description='Record CSV (Indention Test)')
    parser.add_argument('-w', '--width', help='The width of the object')
    parser.add_argument('-r', '--radious', help='The radious of the object')
    args = parser.parse_args()
    
    rclpy.init()
    
    recoder = RecordCSV(
        object_width=args.width,
        object_radious=args.radious,
    )

    rclpy.spin(recoder)

    rclpy.shutdown()

if __name__=='__main__':
    main()