import rclpy
from rclpy.node import Node

from std_msgs.msg import Int16, Int32

class RotarySubscriber(Node):

    def __init__(self):
        super().__init__('rotary_subscriber')
        self.rotary_subscription = self.create_subscription(
            Int16,
            '/esp/rotary_encoder',
            self.rotary_listener_callback,
            10
        )
        self.rotary_previous_value = 0
        self.rotary_rotation_direction = ''
        self.rotary_subscription



        self.ore_subscription = self.create_subscription(
            Int32,
            'esp/optical_rotary_encoder',
            self.ore_listener_callback,
            10
        )

        self.ore_previous_value = 0
        self.ore_rotation_direction = ''
        self.ore_subscription


    def rotary_listener_callback(self, msg):
        if msg.data > self.rotary_previous_value:
            self.rotary_rotation_direction = "Right"
        else:
            self.rotary_rotation_direction = "Left"
        
        self.get_logger().info('Rotary Encoder: {direction} rotation - {value}'.format(direction = self.rotary_rotation_direction, value = msg.data))

        self.rotary_previous_value = msg.data

    def ore_listener_callback(self, msg):
        if msg.data > self.ore_previous_value:
            self.ore_rotation_direction = "Right"
        else:
            self.ore_rotation_direction = "Left"
        
        self.get_logger().info('Optical Rotary Encoder: {direction} rotation - {value}'.format(direction = self.ore_rotation_direction, value = msg.data))

        self.ore_previous_value = msg.data

def main(args=None):
    rclpy.init(args=args)

    rotary_subscriber = RotarySubscriber()

    rclpy.spin(rotary_subscriber)

    rotary_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()