import pyrealsense2 as rs
import numpy as np
import cv2
import torch
import json
import serial
import os

import rclpy
from std_msgs.msg import String


# RealSenseカメラの管理とデータストリームの処理
class RealSenseManager:
    def __init__(self, config):
        self.pipeline = rs.pipeline()
        self.config = config
        self.config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

    # ストリームの開始
    def start_streaming(self):
        self.pipeline.start(self.config)

    # フレームの取得
    def get_frames(self):
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        return depth_frame, color_frame

    # ストリームの停止
    def stop_streaming(self):
        self.pipeline.stop()


# 物体検出と距離の計算
class ObjectDetector:
    def __init__(self):
        self.model = torch.hub.load("ultralytics/yolov5", "yolov5s")
        self.rect_threshold = int(1280 / 8 * 720 / 8)
        self.range_value = 3

    # 距離の計算
    def calculate_distance(self, depth_image, x, y):
        x, y = int(x), int(y)
        region = depth_image[
            y - self.range_value : y + self.range_value + 1,
            x - self.range_value : x + self.range_value + 1,
        ]
        distances = region[region != 0]
        return np.mean(distances) if distances.size != 0 else 0

    # 物体の検出
    def detect_objects(self, img, depth_image):
        img_copy = img.copy()  # imgのコピーを作成
        result = self.model(img_copy)  # コピーに対してYoloの検出を行う
        rendered_image = result.render()  # Yoloの検出結果ありの画像を取得

        detected_objects = []
        for detection in result.xyxy[0]:
            if (detection[2] - detection[0]) * (
                detection[3] - detection[1]
            ) < self.rect_threshold:
                continue
            label = result.names[int(detection[5])]
            x_center = (detection[0] + detection[2]) / 2
            y_center = (detection[1] + detection[3]) / 2
            distance = self.calculate_distance(depth_image, x_center, y_center)
            detected_objects.append((label, distance))
        return detected_objects, rendered_image[0]  # 検出結果と画像の両方を返す


# 白線検出
class WhiteLineDetector:
    # 射影変換を行う関数
    def perspective_transform(self, image):
        src = np.float32(
            [[0 + 400, 0 + 000], [1280 - 400, 0 + 000], [0, 720], [1280, 720]]
        )
        dst = np.float32([[0, 0], [1280, 0], [0, 720], [1280, 720]])
        matrix = cv2.getPerspectiveTransform(src, dst)
        return cv2.warpPerspective(image, matrix, (image.shape[1], image.shape[0]))

    # 白色を検出する関数
    def detect_white(self, image):
        # BGRからHSVへ変換
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # 白色を検出するための閾値
        lower_white = np.array([0, 0, 200], dtype=np.uint8)
        upper_white = np.array([180, 50, 255], dtype=np.uint8)

        # 白色のみ抽出
        mask = cv2.inRange(hsv, lower_white, upper_white)

        return mask

    # ぼかしフィルタを適用する関数
    def apply_blur(self, image):
        # ガウシアンぼかしを適用
        blurred_image = cv2.GaussianBlur(image, (15, 15), 0)
        return blurred_image

    # 白線を検出する関数
    def detect_white_lines(self, image):
        # 元の画像をコピー
        processed_image = image.copy()

        # 処理する領域を画像から切り抜き
        region_of_interest = processed_image[360:720, 0:1280]

        # ぼかしフィルタを適用
        blurred_image = self.apply_blur(region_of_interest)

        # 白色を検出
        mask = self.detect_white(blurred_image)

        # Cannyエッジ検出
        canny = cv2.Canny(mask, 150, 300)

        # Hough変換で直線検出
        lines = cv2.HoughLinesP(
            canny, 1, np.pi / 180, threshold=100, minLineLength=200, maxLineGap=20
        )

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0] + [0, 360, 0, 360]
                angle = np.arctan2(y2 - y1, x2 - x1) * 180.0 / np.pi
                if -10 <= angle <= 10:
                    cv2.line(processed_image, (x1, y1), (x2, y2), (0, 255, 0), 3)
        return processed_image


# 検出データの整理と送信
class DataOrganizer:
    def __init__(self, output_dir, use_serial_communication=False):
        self.output_dir = output_dir
        self.use_serial_communication = use_serial_communication
        if self.use_serial_communication:
            self.ser = serial.Serial("COM4", 115200)

    # 物体と距離の情報を辞書形式で整形する関数
    def organize_data(self, detected_objects):
        detected_data = {}
        for _, obj in enumerate(sorted(detected_objects, key=lambda x: x[1])[:3]):
            detected_data[obj[0]] = round(obj[1] / 1000, 2) if obj[1] != 0 else "NA"

        # 3つ未満の場合はNAで埋める
        while len(detected_data) < 3:
            detected_data[f"NA_{len(detected_data)}"] = "NA"

        return detected_data

    # データの送信
    def send_data(self, detected_data, pub):
        data_to_send = json.dumps(detected_data)
        print(data_to_send)

        # メッセージの作成と送信
        msg = String()
        msg.data = data_to_send
        pub.publish(msg)

        if self.use_serial_communication:
            self.ser.write((data_to_send + "\r").encode("utf-8"))
        with open(f"{self.output_dir}/output.json", "a") as file:
            file.write(data_to_send + "\r")


# 画像の表示
class DisplayManager:
    def __init__(self):
        pass

    # 4分割の画像の表示
    def display_quadrants(self, raw_image, yolo_image, depth_image, white_line_image):
        # Depthの画像をカラーマップに変換
        depth_colormap = cv2.applyColorMap(
            cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET
        )

        # 4つの画像を上下左右に結合
        top_images = np.hstack((raw_image, yolo_image))
        bottom_images = np.hstack((depth_colormap, white_line_image))
        combined = np.vstack((top_images, bottom_images))

        cv2.namedWindow("Quadrants", cv2.WINDOW_NORMAL)
        cv2.imshow("Quadrants", combined)
        cv2.waitKey(1)


# メイン関数
def main():
    USE_SERIAL_COMMUNICATION = False
    OUTPUT_DIR = "./out"
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    # ノードの初期化
    rclpy.init()
    node = rclpy.create_node("publisher_node")

    # パブリッシャの設定
    pub = node.create_publisher(String, "topic", 10)

    config = rs.config()

    realsense_manager = RealSenseManager(config)
    object_detector = ObjectDetector()
    white_line_detector = WhiteLineDetector()
    data_organizer = DataOrganizer(OUTPUT_DIR, USE_SERIAL_COMMUNICATION)

    display_manager = DisplayManager()

    realsense_manager.start_streaming()

    try:
        while True:
            depth_frame, color_frame = realsense_manager.get_frames()
            if not depth_frame or not color_frame:
                continue
            depth_image = np.asanyarray(depth_frame.get_data())
            raw_color_image = np.asanyarray(color_frame.get_data())  # 生の映像を保存
            hole_filling = rs.hole_filling_filter(1)
            depth_frame = hole_filling.process(depth_frame)

            detected_objects, yolo_image = object_detector.detect_objects(
                raw_color_image, depth_image
            )  # 変更された部分

            detected_data = data_organizer.organize_data(detected_objects)
            data_organizer.send_data(detected_data, pub)

            # 白線を検出する
            white_line_image = white_line_detector.detect_white_lines(raw_color_image)

            display_manager.display_quadrants(
                raw_color_image, yolo_image, depth_image, white_line_image
            )

    finally:
        realsense_manager.stop_streaming()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
