import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan

class PIDController:
    def __init__(self, kp, ki, kd):
        # PID coefficients
        self.kp = kp
        self.ki = ki
        self.kd = kd

        # PID error terms
        self.prev_error = 0.0
        self.integral = 0.0

    def compute(self, error, dt):
        # Proportional term
        p = self.kp * error

        # Integral term
        self.integral += error * dt
        i = self.ki * self.integral

        # Derivative term
        derivative = (error - self.prev_error) / dt
        d = self.kd * derivative

        # Update previous error
        self.prev_error = error

        # Calculate PID output
        output = p + i + d
        return output

class ControlPublisher(Node):
    def __init__(self):
        super().__init__('control_publisher')

        # Publisher for throttle and steering commands
        self.throttle_publisher = self.create_publisher(Float32, '/autodrive/f1tenth_1/throttle_command', 10)
        self.steering_publisher = self.create_publisher(Float32, '/autodrive/f1tenth_1/steering_command', 10)

        # Subscriber for LIDAR data
        self.subscription = self.create_subscription(
            LaserScan,
            '/autodrive/f1tenth_1/lidar',
            self.lidar_callback,
            10
        )

        # Timer to periodically publish throttle and steering commands
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.publish_controls)

        # Initialize variables to store track border information
        self.left_border = None
        self.right_border = None
        self.last_time = self.get_clock().now()

        # Initialize PID controller with tuned parameters (to be adjusted for your setup)
        self.pid = PIDController(kp=1.0, ki=0.0, kd=0.4)

    def lidar_callback(self, msg: LaserScan):
        """
        Callback function to process LIDAR data.
        The LIDAR scan provides an array of distances, and we check the left and right borders.
        """
        ranges = msg.ranges
        num_ranges = len(ranges)

        if num_ranges == 0:
            self.get_logger().warn("LIDAR scan contains no data.")
            return

        # Divide the ranges into left and right sides
        left_ranges = ranges[:num_ranges // 3]  # First third for left side
        right_ranges = ranges[-(num_ranges // 3):]  # Last third for right side

        # Find minimum distance in left and right to detect track borders
        self.left_border = min(left_ranges)
        self.right_border = min(right_ranges)

        # self.get_logger().info(f"Left border: {self.left_border}, Right border: {self.right_border}")

    def publish_controls(self):
        """
        Publish throttle and steering commands based on LIDAR data.
        The throttle is constant and steering is adjusted using the PID controller.
        """
        # Throttle command (constant)
        throttle_msg = Float32()
        throttle_msg.data = 0.01  # Constant slow speed
        self.throttle_publisher.publish(throttle_msg)
        # self.get_logger().info(f"Publishing throttle: {throttle_msg.data}")

        # Steering command (PID-controlled)
        if self.left_border is not None and self.right_border is not None:
            # Calculate center offset error
            center_offset = self.right_border - self.left_border

            # Get current time and compute delta time
            current_time = self.get_clock().now()
            dt = (current_time - self.last_time).nanoseconds / 1e9
            self.last_time = current_time

            # Compute the steering angle using PID control
            steering_angle = self.pid.compute(center_offset, dt)

            # Normalize the steering value between [-0.523, 0.523]
            steering_angle = max(-0.523, min(0.523, steering_angle))

            # Publish the steering command
            steering_msg = Float32()
            steering_msg.data = steering_angle
            self.steering_publisher.publish(steering_msg)
            # self.get_logger().info(f"Publishing steering: {steering_msg.data}")
        else:
            # Publish neutral steering if no LIDAR data is available
            neutral_steering = Float32()
            neutral_steering.data = 0.0
            self.steering_publisher.publish(neutral_steering)
            # self.get_logger().info("Publishing neutral steering (0.0)")

def main(args=None):
    rclpy.init(args=args)
    node = ControlPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
