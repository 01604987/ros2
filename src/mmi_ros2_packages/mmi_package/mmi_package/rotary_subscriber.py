import rclpy
from rclpy.node import Node

from std_msgs.msg import Int16

class RotarySubscriber(Node):

    def __init__(self):
        super().__init__('rotary_subscriber')
        self.subscription = self.create_subscription(
            Int16,
            '/esp/rotary_encoder',
            self.listener_callback,
            10
        )
        self.previous_value = 0
        self.rotation_direction = ''
        self.subscription

    def listener_callback(self, msg):
        if msg.data > self.previous_value:
            self.rotation_direction = "Right"
        else:
            self.rotation_direction = "Left"
        
        self.get_logger().info('{direction} rotation - {value}'.format(direction = self.rotation_direction, value = msg.data))

        self.previous_value = msg.data

def main(args=None):
    rclpy.init(args=args)

    rotary_subscriber = RotarySubscriber()

    rclpy.spin(rotary_subscriber)

    rotary_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()