#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float32MultiArray

class WhiteLineDetectionNode(Node):
    def __init__(self):
        super().__init__('white_line_detection_node')
        
        # 画像変換用のブリッジ
        self.bridge = CvBridge()
        
        # 白線検出器の初期化
        self.detector = WhiteLineDetector()
        
        # サブスクライバーの設定
        self.rgb_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.rgb_callback,
            10)
        
        # パブリッシャーの設定
        self.result_pub = self.create_publisher(
            Float32MultiArray,
            '/white_line/detection_result',
            10)
        self.vis_pub = self.create_publisher(
            Image,
            '/white_line/visualization',
            10)
        
        self.get_logger().info("白線検出ノードが起動しました")

    def rgb_callback(self, msg):
        try:
            # ROSメッセージをOpenCV形式に変換
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 白線検出を実行
            result_image, centers, lines = self.detector.detect_white_line(cv_image)
            
            if centers:
                # 検出結果をパブリッシュ
                result_msg = Float32MultiArray()
                # 各中心点の座標を配列に追加
                for center in centers:
                    result_msg.data.extend([float(center[0]), float(center[1])])
                self.result_pub.publish(result_msg)
            
            # 可視化結果をパブリッシュ
            vis_msg = self.bridge.cv2_to_imgmsg(result_image, "bgr8")
            self.vis_pub.publish(vis_msg)
            
        except Exception as e:
            self.get_logger().error(f"画像処理中にエラーが発生しました: {str(e)}")

class WhiteLineDetector:
    def __init__(self):
        pass

    def perspective_transform(self, image):
        """射影変換を行う関数"""
        src = np.float32(
            [[0 + 400, 0 + 000], [1280 - 400, 0 + 000], [0, 720], [1280, 720]]
        )
        dst = np.float32([[0, 0], [1280, 0], [0, 720], [1280, 720]])
        matrix = cv2.getPerspectiveTransform(src, dst)
        return cv2.warpPerspective(image, matrix, (image.shape[1], image.shape[0]))

    def detect_white(self, image):
        """白色を検出する関数"""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_white = np.array([0, 0, 200], dtype=np.uint8)
        upper_white = np.array([180, 50, 255], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower_white, upper_white)
        return mask

    def apply_blur(self, image):
        """ぼかしフィルタを適用する関数"""
        blurred_image = cv2.GaussianBlur(image, (15, 15), 0)
        return blurred_image

    def detect_white_lines(self, image):
        """白線を検出する関数"""
        processed_image = image.copy()
        region_of_interest = processed_image[360:720, 0:1280]
        blurred_image = self.apply_blur(region_of_interest)
        mask = self.detect_white(blurred_image)
        canny = cv2.Canny(mask, 150, 300)
        lines = cv2.HoughLinesP(
            canny, 1, np.pi / 180, threshold=100, minLineLength=200, maxLineGap=20
        )
        return lines

    def filter_horizontal_lines(self, lines):
        """水平に近い白線のみを残す関数"""
        horizontal_lines = []
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                angle = np.arctan2(y2 - y1, x2 - x1) * 180.0 / np.pi
                if -10 <= angle <= 10:
                    horizontal_lines.append(line)
        return horizontal_lines

    def calculate_line_centers(self, lines):
        """白線の中心座標を求める関数"""
        centers = []
        for line in lines:
            x1, y1, x2, y2 = line[0]
            centers.append(((x1 + x2) // 2, (y1 + y2) // 2))
        return centers

    def draw_lines(self, image, lines):
        """画像に検出した直線を描画する関数"""
        for line in lines:
            x1, y1, x2, y2 = line[0] + [0, 360, 0, 360]
            cv2.line(image, (x1, y1), (x2, y2), (0, 255, 0), 3)
        return image

    def detect_white_line(self, image):
        """白線検出のメイン関数"""
        transformed_image = self.perspective_transform(image)
        lines = self.detect_white_lines(transformed_image)
        horizontal_lines = self.filter_horizontal_lines(lines)
        centers = self.calculate_line_centers(horizontal_lines)
        result_image = self.draw_lines(transformed_image, horizontal_lines)
        
        for center in centers:
            cv2.circle(result_image, center, 5, (0, 0, 255), -1)
            position_text = f"Position: {center}"
            cv2.putText(result_image, position_text, (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        return result_image, centers, horizontal_lines

def main(args=None):
    rclpy.init(args=args)
    node = WhiteLineDetectionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()