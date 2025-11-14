import rclpy as rp
from rclpy.node import Node
from turtlesim.msg import Pose
import time

class TurtlesimSubscriber(Node):
    def __init__(self):
        super().__init__('turtlesim_subscriber')

        # pose를 저장할 변수
        self.latest_pose = None

        # subscriber 생성
        self.subscription = self.create_subscription(
            Pose,                    # 메시지 타입
            '/turtle1/pose',        # 토픽 이름
            self.callback,           # 콜백 함수
            10                       # QoS
        )

    def callback(self, msg: Pose):
        """토픽이 들어올 때마다 실행됨"""
        self.latest_pose = msg  # 최신 pose 저장

        # 콘솔 출력 (원하면 제거 가능)
        current_time = time.strftime("%H:%M:%S", time.localtime())
        print(f"[{current_time}] X={msg.x:.2f}, Y={msg.y:.2f}, Theta={msg.theta:.2f} rad")


def main(args=None):
    rp.init(args=args)
    node = TurtlesimSubscriber()
    rp.spin(node)
    node.destroy_node()
    rp.shutdown()


if __name__ == '__main__':
    main()
