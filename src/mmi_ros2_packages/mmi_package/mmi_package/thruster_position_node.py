import rclpy
from rclpy.node import Node

from std_msgs.msg import Int16, Int32, Float32, Bool

class ThrusterPosition(Node):

    def __init__(self):
        super().__init__('thruster_position')

        self.unity_thruster_calibration = self.create_subscription(
            Bool,
            'unity/calibrate/thruster',
            self.thruster_calibration_callback,
            10
        )

        self.min_rotary_val = 0
        self.max_rotary_val = 0
        self.distance = 0
        self.calibrate_flag = 0
        self.calibrated_min = None
        self.calibrated_max = None
        self.position_normalized = Float32()

        self.ore_subscription = self.create_subscription(
            Int32,
            'esp/optical_rotary_encoder',
            self.ore_listener_callback,
            10
        )
        self.ore_previous_value = 0
        self.ore_subscription


        self.thruster_position_publisher = self.create_publisher(
            Float32,
            'esp/thruster_position',
            10
        )


    def thruster_calibration_callback(self, msg):
        self.calibrate_flag = msg.data

        if (self.calibrate_flag):
            self.get_logger().info("Calibration initiated".format())

            # initialize calibration values
            self.calibrated_min = None
            self.calibrated_max = None

        elif (self.calibrated_max is not None and self.calibrated_min is not None):
            # assign calibrated values

            self.distance = self.calibrated_max - self.calibrated_min
            self.max_rotary_val = self.calibrated_max
            self.min_rotary_val = self.calibrated_min
            self.get_logger().info("Calibration complete, new min: {} | new max: {} | new distance: {}".format(self.calibrated_min, self.calibrated_max, self.distance))



    def calibrate(self, msg):
        if self.calibrated_min is None:
            self.calibrated_min = msg.data
        elif msg.data < self.calibrated_min:
            self.calibrated_min = msg.data


        if self.calibrated_max is None:
            self.calibrated_max = msg.data
        elif msg.data > self.calibrated_max:
            self.calibrated_max = msg.data
                

    def calculate_position(self, msg):
        self.position_normalized.data = (msg.data - self.min_rotary_val) / (self.distance)
        self.thruster_position_publisher.publish(self.position_normalized)
        self.get_logger().info("Normalized Pos: {}".format(self.position_normalized))



    def ore_listener_callback(self, msg):
        
        if self.calibrate_flag:
            self.calibrate(msg)          


        if self.distance:
            self.calculate_position(msg)


        if msg.data > self.ore_previous_value:
            self.ore_rotation_direction = "Right"
        else:
            self.ore_rotation_direction = "Left"
        
        #self.get_logger().info('Optical Rotary Encoder: {direction} rotation - {value}'.format(direction = self.ore_rotation_direction, value = msg.data))

        self.ore_previous_value = msg.data

def main(args=None):
    rclpy.init(args=args)

    rotary_position = ThrusterPosition()

    rclpy.spin(rotary_position)

    rotary_position.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()