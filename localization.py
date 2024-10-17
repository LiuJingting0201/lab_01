import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist

class Localization(Node):
    def __init__(self):
        super().__init__('localization')
        self.subscription = self.create_subscription(Twist, '/cmd_topic', self.listener_callback, 10)
        self.publisher_ = self.create_publisher(Pose, '/pose', 10)

        # Initial position
        self.x = 0.0
        self.y = 0.0
        self.last_time = self.get_clock().now()  # Keep track of time

    def listener_callback(self, msg):
        # Get the current time and calculate time elapsed since last update
        current_time = self.get_clock().now()
        time_delta = (current_time - self.last_time).nanoseconds / 1e9  # Convert nanoseconds to seconds
        self.last_time = current_time

        # Update position based on velocity and time delta
        delta_x = msg.linear.x * time_delta
        delta_y = msg.linear.y * time_delta
        self.x += delta_x
        self.y += delta_y

        # Round x and y to nearest integer, then convert back to float
        self.x = float(round(self.x))
        self.y = float(round(self.y))

        # Create and publish the Pose message
        pose_msg = Pose()
        pose_msg.position.x = self.x
        pose_msg.position.y = self.y
        pose_msg.position.z = 0.0  # Assuming motion on 2D plane
        self.publisher_.publish(pose_msg)

        # Log the updated position
        self.get_logger().info(f'Published position: x={self.x}, y={self.y}')

def main(args=None):
    rclpy.init(args=args)
    localization = Localization()
    rclpy.spin(localization)
    localization.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

