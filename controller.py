import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)  # 1 Hz timer
        self.state = 0  # 0: X+, 1: Y+, 2: X-, 3: Y-
        self.counter = 1  # N starts at 1
        self.seconds_passed = 0  # Time spent in the current direction

    def timer_callback(self):
        msg = Twist()

        # Control logic for different directions
        if self.state == 0:  # Move along X-axis (positive)
            msg.linear.x = 1.0
            msg.linear.y = 0.0
            self.get_logger().info(f'Moving along X+ for {self.counter} seconds.')
        elif self.state == 1:  # Move along Y-axis (positive)
            msg.linear.x = 0.0
            msg.linear.y = 1.0
            self.get_logger().info(f'Moving along Y+ for {self.counter} seconds.')
        elif self.state == 2:  # Move along X-axis (negative)
            msg.linear.x = -1.0
            msg.linear.y = 0.0
            self.get_logger().info(f'Moving along X- for {self.counter} seconds.')
        elif self.state == 3:  # Move along Y-axis (negative)
            msg.linear.x = 0.0
            msg.linear.y = -1.0
            self.get_logger().info(f'Moving along Y- for {self.counter} seconds.')

        self.publisher_.publish(msg)
        self.seconds_passed += 1

        # After N seconds, switch direction
        if self.seconds_passed >= self.counter:
            self.state += 1
            if self.state == 4:
                self.state = 0#(self.state + 1) % 4  # Cycle through states (0, 1, 2, 3)
            self.seconds_passed = 0  # Reset the timer for the next state

            # After a full cycle (X+, Y+, X-, Y-), increment N
            if self.state == 0:
                self.counter += 1
            # Increment N after each direction change
            #self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

