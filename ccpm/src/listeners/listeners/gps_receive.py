import rclpy
from rclpy.node import Node

from ccpm_msgs.msg import GPSMessage


class GPSSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')

        self.message_type = GPSMessage
        self.topic_name = '/gps/receive'
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

    gps_subscriber = GPSSubscriber()

    rclpy.spin(gps_subscriber)

    gps_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()