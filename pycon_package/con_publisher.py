import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class ConPublisher(Node):
    def __init__(self):
        super().__init__('con_publisher')
        # /turtle1/cmd_vel 토픽 퍼블리셔 생성
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
    
    def move(self, linear=0.0, angular=0.0):
        """
        거북이 이동 함수
        linear : 직진 속도
        angular : 회전 속도
        """
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.publisher.publish(msg)
        self.get_logger().info(f"Move command -> Linear: {linear}, Angular: {angular}")

def main(args=None):
    rclpy.init(args=args)
    node = ConPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
