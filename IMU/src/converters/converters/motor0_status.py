import rclpy
import os
from rclpy.node import Node

from ccpm_msgs.msg import XP0Powertrain


class Motor0Converter(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')

        self.csv_directory = '/home/hugobaptista/ros2/IMU/csv_nodes'
        self.message_type = XP0Powertrain
        self.topic_name = '/motor0/status'

        self.csv_filename = self.detect_csv()
        self.subscription = self.create_subscription(
            self.message_type,
            self.topic_name,
            self.listener_callback,
            10)
        self.subscription

    def detect_csv(self):
        filename = f'{self.topic_name[1:].replace("/","_")}.csv'
        if filename not in os.listdir(f'{self.csv_directory}'):                                     # if the csv file does not exist
            with open(f'{self.csv_directory}/{filename}','w') as file:                              # create a new csv file
                file.write(','.join(self.message_type.get_fields_and_field_types().keys())+'\n')    # and add the csv header
        return filename

    def listener_callback(self, msg):
        with open(f'{self.csv_directory}/{self.csv_filename}','a') as file:
            file.write(','.join([str(getattr(msg, field)) for field in self.message_type.get_fields_and_field_types().keys()])+'\n')

def main(args=None):
    rclpy.init(args=args)

    motor0_converter = Motor0Converter()

    rclpy.spin(motor0_converter)

    motor0_converter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()