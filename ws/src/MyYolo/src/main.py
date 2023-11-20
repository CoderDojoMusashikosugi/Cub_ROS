#!/home/musashikosugi/Cub/RealSense/MyYolo-MakeClass/.venv/bin/python
#! coding:utf-8

import pyrealsense2 as rs
import numpy as np
import cv2
import torch
import json
import serial
import os


# RealSenseカメラの管理とデータストリームの処理
class RealSenseManager:
    def __init__(self, config):
        self.pipeline = rs.pipeline()
        self.config = config
        self.config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 5)
        self.config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 5)
        self.align = rs.align(rs.stream.color)  # RGB画像に合わせて深度画像をアライメントする

    # ストリームの開始
    def start_streaming(self):
        self.pipeline.start(self.config)

    # アライメントされたフレームの取得
    def get_frames(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        aligned_depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        return aligned_depth_frame, color_frame

    # ストリームの停止
    def stop_streaming(self):
        self.pipeline.stop()


# 物体検出クラス
class ObjectDetector:
    def __init__(self):
        self.model = torch.hub.load("ultralytics/yolov5", "yolov5s")
        self.rect_threshold = int(1280 / 8 * 720 / 8)
        self.confidence_threshold = 0.5  # 確信度のしきい値を設定
        self.model.conf = self.confidence_threshold

    # 検出された物体のフィルタリング
    def filter_detections(self, detections):
        filtered_detections = []
        for detection in detections:
            area = (detection[2] - detection[0]) * (detection[3] - detection[1])

            # 面積と確信度に基づいてフィルタリング
            if area >= self.rect_threshold:
                filtered_detections.append(detection)

        return filtered_detections

    # 物体の検出
    def detect_objects(self, img):
        # img_copy = img.copy()

        # yolo実行
        result = self.model(img)
        detected_objects = []

        # フィルタリングされた検出結果を取得
        filtered_detections = self.filter_detections(result.xyxy[0])

        # フィルタリングされた結果を処理
        for detection in filtered_detections:
            label = result.names[int(detection[5])]
            x_center = (detection[0] + detection[2]) / 2
            y_center = (detection[1] + detection[3]) / 2
            detected_objects.append((label, x_center, y_center))

        return detected_objects, result.render()[0]  # 検出結果と画像の両方を返す


# 距離計算クラス
class DistanceCalculator:
    def __init__(self):
        self.range_value = 1

    # 距離の計算
    def calculate_distance(self, depth_image, x, y):
        x, y = int(x), int(y)
        region = depth_image[
            y - self.range_value : y + self.range_value + 1,
            x - self.range_value : x + self.range_value + 1,
        ]
        distances = region[region != 0] / 1000
        return np.mean(distances) if distances.size != 0 else 0


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

        return lines

    # 水平に近い白線のみを残す関数
    def filter_horizontal_lines(self, lines):
        horizontal_lines = []
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                angle = np.arctan2(y2 - y1, x2 - x1) * 180.0 / np.pi
                if -10 <= angle <= 10:
                    horizontal_lines.append(line)
        return horizontal_lines

    # 白線の中心座標を求める関数
    def calculate_line_centers(self, lines):
        centers = []
        for line in lines:
            x1, y1, x2, y2 = line[0]
            centers.append(((x1 + x2) // 2, (y1 + y2) // 2))
        return centers

    # 画像に検出した直線を描画する関数
    def draw_lines(self, image, lines):
        for line in lines:
            x1, y1, x2, y2 = line[0] + [0, 360, 0, 360]
            cv2.line(image, (x1, y1), (x2, y2), (0, 255, 0), 3)
        return image


# 検出データの整理と送信
class DataOrganizer:
    def __init__(self, output_dir, use_serial_communication=False):
        self.output_dir = output_dir
        self.use_serial_communication = use_serial_communication
        if self.use_serial_communication:
            self.ser = serial.Serial("COM4", 115200)

        self.detected_classes = ["person", "dog", "cat"]
        self.distance_threshold = 1.0

        self.x_range = [0 + 200, 1280 - 200]
        self.y_range = [0 + 100, 720 - 100]

    # YOLOとライントラックの結果を分析する関数
    def analyze_yolo_results(self, yolo_results, x_range, y_range):
        for result in yolo_results:
            if result[0] in self.detected_classes:
                distance = result[3]
                x, y = result[1], result[2]
                print(distance, x, y)
                if (
                    distance < self.distance_threshold
                    and x_range[0] <= x <= x_range[1]
                    and y_range[0] <= y <= y_range[1]
                ):
                    return True
        return False

    # 白線検出の結果を分析する関数
    def analyze_linetrack_results(self, linetrack_results, distance_threshold, x_range):
        for result in linetrack_results:
            distance = result["distance"]
            x = result["x_coordinate"]
            if distance < distance_threshold and x_range[0] <= x <= x_range[1]:
                return True
        return False

    # 物体と距離の情報を辞書形式で整形する関数
    def organize_data(self, detected_objects):
        detected_data = {}
        for _, obj in enumerate(sorted(detected_objects, key=lambda x: x[1])[:3]):
            # Tensorをfloat型に変換してからround関数を適用
            distance = obj[1].item() if obj[1] != 0 else 0
            detected_data[obj[0]] = round(distance / 1000, 2) if distance != 0 else "NA"

        # 3つ未満の場合はNAで埋める
        while len(detected_data) < 3:
            detected_data[f"NA_{len(detected_data)}"] = "NA"

        return detected_data

    # データの送信
    def send_data(self, detected_data):
        data_to_send = json.dumps(detected_data)
        print(data_to_send)
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

    config = rs.config()

    realsense_manager = RealSenseManager(config)
    object_detector = ObjectDetector()
    distance_calculator = DistanceCalculator()
    white_line_detector = WhiteLineDetector()
    data_organizer = DataOrganizer(OUTPUT_DIR, USE_SERIAL_COMMUNICATION)

    display_manager = DisplayManager()

    realsense_manager.start_streaming()

    try:
        while True:
            # 画像取得
            depth_frame, color_frame = realsense_manager.get_frames()

            # 画像が取得できなかった場合はスキップ
            if not depth_frame or not color_frame:
                continue

            # 深度画像の穴埋め
            hole_filling = rs.hole_filling_filter(1)
            depth_frame = hole_filling.process(depth_frame)

            # 深度画像をnumpy配列に変換
            depth_image = np.asanyarray(depth_frame.get_data())

            # カラー画像をnumpy配列に変換
            color_image = np.asanyarray(color_frame.get_data())

            # 生のカラー画像を保存
            raw_color_image = color_image.copy()

            # 物体検出の実行
            detected_objects, yolo_image = object_detector.detect_objects(
                color_image
            )  # (Label,x,y),image

            # 検出された物体の距離を計算
            detected_objects_with_distance = []
            for label, x_center, y_center in detected_objects:
                # 中心座標をtensor形式からint型に変換
                x_center = int(x_center)
                y_center = int(y_center)

                distance = distance_calculator.calculate_distance(
                    depth_image, x_center, y_center
                )
                detected_objects_with_distance.append(
                    (label, x_center, y_center, distance)
                )

            # # 検出データの整理と送信
            # detected_data = data_organizer.organize_data(detected_objects)
            # data_organizer.send_data(detected_data)

            # 白線を検出する
            white_lines = white_line_detector.detect_white_lines(color_image)

            # 水平に近い白線のみを残す
            horizontal_lines = white_line_detector.filter_horizontal_lines(white_lines)

            # 白線の中心座標を求める
            line_centers = white_line_detector.calculate_line_centers(horizontal_lines)
            print(line_centers)

            # 白線の座標から距離を計算する
            for x, y in line_centers:
                distance = (
                    distance_calculator.calculate_distance(depth_image, x, y)
                )
                print(f"white_line_distance: {distance}")

            # 画像に検出した直線を描画
            white_line_image = white_line_detector.draw_lines(
                color_image, horizontal_lines
            )

            # 画像の表示
            display_manager.display_quadrants(
                raw_color_image, yolo_image, depth_image, white_line_image
            )

            # この辺からデータ整理

            # YOLOの結果を整理
            obstacle_exists = data_organizer.analyze_yolo_results(
                detected_objects_with_distance,
                data_organizer.x_range,
                data_organizer.y_range,
            )

            print(obstacle_exists)

    finally:
        realsense_manager.stop_streaming()


if __name__ == "__main__":
    main()
