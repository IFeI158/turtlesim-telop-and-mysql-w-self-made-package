import sys
import threading
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QHBoxLayout
import rclpy
from pycon_package.con_publisher import ConPublisher
from pycon_package.con_subscriber import TurtlesimSubscriber
from std_srvs.srv import Empty
from pycon_package.db_helper import DB, DB_CONFIG


class TurtleGUI(QWidget):
    def __init__(self, publisher_node):
        super().__init__()
        self.pub_node = publisher_node
        self.setWindowTitle("Turtle Controller")

        # ★ subscriber 노드 생성
        self.sub_node = TurtlesimSubscriber()

        # ★ subscriber를 spin할 별도의 스레드 생성
        self.sub_thread = threading.Thread(target=self.spin_subscriber, daemon=True)
        self.sub_thread.start()

        layout = QVBoxLayout()

        # 위쪽 버튼
        btn_up = QPushButton("↑")
        btn_up.clicked.connect(lambda: self.pub_node.move(linear=1.0))
        layout.addWidget(btn_up)

        # 좌/우 버튼
        h_layout = QHBoxLayout()
        btn_left = QPushButton("←")
        btn_left.clicked.connect(lambda: self.pub_node.move(angular=1.0))
        btn_right = QPushButton("→")
        btn_right.clicked.connect(lambda: self.pub_node.move(angular=-1.0))
        h_layout.addWidget(btn_left)
        h_layout.addWidget(btn_right)
        layout.addLayout(h_layout)

        # 아래쪽 버튼
        btn_down = QPushButton("↓")
        btn_down.clicked.connect(lambda: self.pub_node.move(linear=-1.0))
        layout.addWidget(btn_down)

        # Reset 버튼
        btn_reset = QPushButton("Reset")
        btn_reset.clicked.connect(self.reset_turtle)
        layout.addWidget(btn_reset)

        # Save 버튼
        btn_save = QPushButton("Save Pose")
        btn_save.clicked.connect(self.save_pose)
        layout.addWidget(btn_save)

        self.setLayout(layout)


    def spin_subscriber(self):
        rclpy.spin(self.sub_node)

    def reset_turtle(self):
        cli = self.pub_node.create_client(Empty, '/reset')
        if not cli.wait_for_service(timeout_sec=2.0):
            print("Reset service not available")
            return
        req = Empty.Request()
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(self.pub_node, future)
        print("Turtle reset!")

    def save_pose(self):
        pose = self.sub_node.latest_pose

        if pose is None:
            print("아직 pose를 받지 못했습니다.")
            return

        from datetime import datetime
        current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        # DB 객체 생성
        db = DB(**DB_CONFIG)

        # INSERT 실행
        ok = db.db_pose(
            id=1,
            x=pose.x,
            y=pose.y,
            theta=pose.theta,
            time=current_time
        )

        if ok:
            print(f"[DB 저장 성공] x={pose.x:.2f}, y={pose.y:.2f}, theta={pose.theta:.2f}")
        else:
            print("[DB 저장 실패]")


def main(args=None):
    rclpy.init(args=args)

    publisher_node = ConPublisher()

    app = QApplication(sys.argv)
    gui = TurtleGUI(publisher_node)
    gui.show()

    try:
        sys.exit(app.exec_())
    finally:
        publisher_node.destroy_node()
        gui.sub_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
