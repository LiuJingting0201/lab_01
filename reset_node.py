import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
import math

class ResetNode(Node):
    def __init__(self):
        super().__init__('reset_node')
        # 订阅 /pose 话题
        self.subscription = self.create_subscription(Pose, '/pose', self.pose_callback, 10)
        # 发布 /reset 话题
        self.publisher_ = self.create_publisher(Bool, '/reset', 10)

    def pose_callback(self, pose_msg):
        # 计算距离，sqrt(x^2 + y^2)
        x = pose_msg.position.x
        y = pose_msg.position.y
        distance = math.sqrt(x ** 2 + y ** 2)

        # 如果距离大于 6.0 米，发布 True 值到 /reset
        if distance > 6.0:
            reset_msg = Bool()
            reset_msg.data = True  # 或者 False，根据你需要
            self.publisher_.publish(reset_msg)
            self.get_logger().info(f'Distance from origin is {distance:.2f}m, publishing reset: {reset_msg.data}')

def main(args=None):
    rclpy.init(args=args)
    reset_node = ResetNode()
    rclpy.spin(reset_node)
    reset_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

