import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel
from PyQt5.QtCore import QTimer, Qt

class CmdVelDisplay(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()
        self.initROS()
        
    def initUI(self):
        self.setWindowTitle('Cmd Vel Display')
        
        layout = QVBoxLayout()

        # 画面サイズを取得
        screen = QApplication.primaryScreen().geometry()
        width = screen.width() // 3  # 画面幅の1/3
        height = screen.height()
        
        # ウィンドウを画面の左側1/3に配置
        self.setGeometry(0, 0, width, height)
        
        
        self.linear_label = QLabel('Linear: 0.0', self)
        self.linear_label.setStyleSheet('font-size: 40px;')
        self.angular_label = QLabel('Angular: 0.0', self)
        self.angular_label.setStyleSheet('font-size: 40px;')
        layout.addWidget(self.linear_label)
        layout.addWidget(self.angular_label)
        
        self.setLayout(layout)
        
        # タイマーをセットアップ
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_ros)
        self.timer.start(100)  # 100ミリ秒ごとに更新
        
    def initROS(self):
        rclpy.init(args=None)
        self.node = Node('cmd_vel_display')
        self.subscription = self.node.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        self.twist_msg = Twist()
        
    def cmd_vel_callback(self, msg):
        self.twist_msg = msg
        
    def update_ros(self):
        rclpy.spin_once(self.node, timeout_sec=0)
        self.update_labels()
        
    def update_labels(self):
        linear_x = round(self.twist_msg.linear.x, 2)
        angular_z = round(self.twist_msg.angular.z, 2)
        self.linear_label.setText(f'Linear: {linear_x}')
        self.angular_label.setText(f'Angular: {angular_z}')
        
    def closeEvent(self, event):
        self.node.destroy_node()
        rclpy.shutdown()

def main(arg=None):
    app = QApplication(sys.argv)
    ex = CmdVelDisplay()
    ex.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
