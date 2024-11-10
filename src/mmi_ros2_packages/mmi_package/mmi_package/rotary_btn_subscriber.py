import rclpy
from rclpy.node import Node

from std_msgs.msg import Int16

class ButtonSubscriber(Node):

    def __init__(self):
        super().__init__('rotary_btn_subscriber')
        self.subscription = self.create_subscription(
            Int16,
            '/esp/rotary_button',
            self.listener_callback,
            10
        )
        self.subscription

    def listener_callback(self, msg):
        output = "pressed" if msg.data else "released"
        self.get_logger().info('Button {state}'.format(state = output))
        #self.get_logger().info('Button state: {}'.format(msg.data))


def main(args=None):
    rclpy.init(args=args)

    button_subscriber = ButtonSubscriber()

    rclpy.spin(button_subscriber)

    button_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()