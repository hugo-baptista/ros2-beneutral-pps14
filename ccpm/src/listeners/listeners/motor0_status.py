import rclpy
from rclpy.node import Node

from ccpm_msgs.msg import XP0Powertrain


class Motor0Subscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')

        self.message_type = XP0Powertrain
        self.topic_name = '/motor0/status'
        self.msg_num=0

        self.subscription = self.create_subscription(
            self.message_type,
            self.topic_name,
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
        self.msg_num=self.msg_num+1
        field_value_pairs = [f'{field}: {getattr(msg, field)}' for field in self.message_type.get_fields_and_field_types().keys()]
        self.get_logger().info(f'I heard message #{self.msg_num}:\n - '+'\n - '.join(field_value_pairs)+'\n')


def main(args=None):
    rclpy.init(args=args)

    motor0_subscriber = Motor0Subscriber()

    rclpy.spin(motor0_subscriber)

    motor0_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()