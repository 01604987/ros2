import rclpy
import time
from rclpy.node import Node

from std_msgs.msg import Float32

class AudioInputFilterNode(Node):
    def __init__(self):
        super().__init__('audio_input_filter_node')
        
        self.hcsr04_subscription = self.create_subscription(
            Float32,
            'esp/hcsr04',
            self.hcsr04_listener_callback,
            10
        )
        
        self.hcsr04_publisher = self.create_publisher(
            Float32,
            'esp/filtered/hcsr04',
            10
        )

        self.linear_pot_subscription = self.create_subscription(
            Float32,
            'esp/linear_pot',
            self.linear_pot_listener_callback,
            10
        )
        self.linear_pot_publisher = self.create_publisher(
            Float32,
            'esp/filtered/linear_pot',
            10
        )
        


        # for rounding decimal values
        self.round = 3 
        # Exponential Moving Average parameters
        self.alpha = 0.1  # Smoothing factor (adjust as needed)
        self.filtered_hcsr04_value = None
        self.prev_filtered_hcsr04_value = None
        self.filtered_linear_pot_value = None
        self.prev_filtered_linear_pot_value = None
        self.raw_hscr04 = None
        self.raw_linear_pot = None

        
        # Timeout mechanism
        self.hcsr04_last_message_time = time.time()
        self.linear_pot_last_message_time = time.time()
        self.timeout = 0.2  # Timeout in seconds
        
        # Timer for periodic publishing
        self.timer = self.create_timer(0.01, self.publish_filtered_values)  # Publish every 10ms

        self.get_logger().info('Audio Input Filter Node initialized')

    def hcsr04_listener_callback(self, msg):
        self.raw_hscr04 = msg.data  # Raw data from the sensor
        
        # Update the last message time
        self.hcsr04_last_message_time = time.time()
        
        # Apply Exponential Moving Average (EMA) filter
        if self.filtered_hcsr04_value is None:
            self.filtered_hcsr04_value = round(self.raw_hscr04, self.round)
        else:
            self.filtered_hcsr04_value = round(self.alpha * self.raw_hscr04 + (1 - self.alpha) * self.filtered_hcsr04_value, self.round)
        
        #self.get_logger().info(f"HCSR04: Raw: {self.raw_hscr04:.2f} cm, Filtered: {self.filtered_hcsr04_value:.2f} cm")

    def linear_pot_listener_callback(self, msg):
        self.raw_linear_pot = msg.data  # Raw data from the sensor
        
        # Update the last message time
        self.linear_pot_last_message_time = time.time()
        
        # Apply Exponential Moving Average (EMA) filter
        if self.filtered_linear_pot_value is None:
            self.filtered_linear_pot_value = round(self.raw_linear_pot, self.round)
        else:
            self.filtered_linear_pot_value = round(self.alpha * self.raw_linear_pot + (1 - self.alpha) * self.filtered_linear_pot_value, self.round)
        
        #self.get_logger().info(f"Linear Pot: Raw: {self.raw_linear_pot:.2f}, Filtered: {self.filtered_linear_pot_value:.2f}")


    def publish_filtered_values(self):
        current_time = time.time()
        
        # Handle timeout for HCSR04
        if current_time - self.hcsr04_last_message_time > self.timeout:
            if self.filtered_hcsr04_value is not None:
                # Gradually decay the filtered value toward 0
                self.filtered_hcsr04_value = round(self.alpha * self.raw_hscr04 + (1 - self.alpha) * self.filtered_hcsr04_value, self.round)
                

        # Handle timeout for Linear Pot
        if current_time - self.linear_pot_last_message_time > self.timeout:
            if self.filtered_linear_pot_value is not None:
                # Gradually decay the filtered value toward 0
                self.filtered_linear_pot_value = round(self.alpha * self.raw_linear_pot + (1 - self.alpha) * self.filtered_linear_pot_value, self.round)




        # Publish filtered HCSR04 value
        if self.filtered_hcsr04_value is not None:
            if (self.filtered_hcsr04_value != self.prev_filtered_hcsr04_value):
                filtered_hcsr04_msg = Float32()
                filtered_hcsr04_msg.data = self.filtered_hcsr04_value
                self.hcsr04_publisher.publish(filtered_hcsr04_msg)
                self.prev_filtered_hcsr04_value = self.filtered_hcsr04_value
                self.get_logger().info(f"HCSR04: Raw: {self.raw_hscr04:.6f} cm, Filtered: {self.filtered_hcsr04_value} cm")



        # Publish filtered Linear Pot value
        if self.filtered_linear_pot_value is not None:
            if (self.filtered_linear_pot_value != self.prev_filtered_linear_pot_value):
                filtered_linear_pot_msg = Float32()
                filtered_linear_pot_msg.data = self.filtered_linear_pot_value
                self.linear_pot_publisher.publish(filtered_linear_pot_msg)
                self.prev_filtered_linear_pot_value = self.filtered_linear_pot_value
                self.get_logger().info(f"Linear Pot: Raw: {self.raw_linear_pot:.6f}, Filtered: {self.filtered_linear_pot_value}")



def main(args=None):
    rclpy.init(args=args)

    node = AudioInputFilterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
