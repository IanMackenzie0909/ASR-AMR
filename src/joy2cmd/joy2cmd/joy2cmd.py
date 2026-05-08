import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class Joy2Cmd(Node):

    def __init__(self):
        super().__init__('joy2cmd')

        # 訂閱 /joy topic
        self.subscription = self.create_subscription(Joy,'/joy',self.joy_callback,10)

        # 發布 /cmd_vel
        self.publisher = self.create_publisher(Twist,'/cmd_vel', 10)

        self.get_logger().info("joy2cmd node started")

    def joy_callback(self, msg):

        twist = Twist()

        # 左搖桿控制
        # axes[1] 前後
        # axes[0] 左右

        twist.linear.x = msg.axes[4] * 0.15
        twist.linear.y = msg.axes[3] * 0.15
        twist.angular.z = msg.axes[0] * 0.7
        # msg.buttons[5]
        # msg.buttons[4]

        self.publisher.publish(twist)

        self.get_logger().info(f'linear_x: {twist.linear.x:.2f},linear_y: {twist.linear.y:.2f}, angular: {twist.angular.z:.2f}')


def main(args=None):

    rclpy.init(args=args)
    node = Joy2Cmd()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()