import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
import tf_transformations  # For quaternion to Euler conversion
import math

class PositionOrientationPublisher(Node):
    def __init__(self):
        super().__init__('position_orientation_publisher')

        # Subscriber for positional data from the /autodrive/f1tenth_1/ips topic
        self.position_subscription = self.create_subscription(
            Point,
            '/autodrive/f1tenth_1/ips',
            self.position_callback,
            10
        )

        # Subscriber for orientation data from the /autodrive/f1tenth_1/imu topic
        self.orientation_subscription = self.create_subscription(
            Imu,
            '/autodrive/f1tenth_1/imu',
            self.orientation_callback,
            10
        )

        # Publisher for the /bot_pose topic
        self.pose_publisher = self.create_publisher(Float32MultiArray, '/bot_pose', 10)

        # Initialize variables to store the latest positional and orientation data
        self.position = None
        self.yaw = None

        # Timer to publish data at regular intervals
        self.timer = self.create_timer(0.1, self.publish_data)  # 1 second interval

    def position_callback(self, msg: Point):
        """
        Callback function to process positional data.
        """
        self.position = (msg.x, msg.y)

    def orientation_callback(self, msg: Imu):
        """
        Callback function to process orientation data from the IMU.
        Converts quaternion to Euler angles and extracts the yaw (rotation around z-axis).
        """
        orientation_q = msg.orientation
        quaternion = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]

        # Convert quaternion to Euler angles (roll, pitch, yaw)
        _, _, yaw = tf_transformations.euler_from_quaternion(quaternion)
        
        # Store yaw (rotation around z-axis)
        self.yaw = yaw

    def publish_data(self):
        """
        Publishes the latest position and orientation data as Float32MultiArray.
        """
        if self.position is not None and self.yaw is not None:
            # Convert yaw to degrees for easier interpretation
            yaw_degrees = math.degrees(self.yaw)

            # Create a Float32MultiArray message
            msg = Float32MultiArray()
            msg.data = [self.position[0], self.position[1], yaw_degrees]

            # Publish the message
            self.pose_publisher.publish(msg)
            self.get_logger().info(f"Pos: [{self.position[0]:.2f}, {self.position[1]:.2f}] | Ori {yaw_degrees:.2f}°")

def main(args=None):
    rclpy.init(args=args)
    node = PositionOrientationPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
