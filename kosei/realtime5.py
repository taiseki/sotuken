import cv2
import torch
import numpy as np
from ultralytics import YOLO
import threading

#Moto
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3

#ROS2 init
rclpy.init()
node = YOLONode()


# カメラの焦点距離と物体の実際の高さ
focal_length = 500  # 必要に応じてキャリブレーションで変更
Hreal = 10.0  # cm

# カメラの画角（仮定値）
horizontal_fov = 60.0  # 水平方向の視野角 (度)
vertical_fov = 45.0    # 垂直方向の視野角 (度)

# GPUを使用する設定
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

# YOLOv8モデルの読み込み (軽量なモデル)
model = YOLO("yolov8n.pt").to(device)

# USBカメラのキャプチャ設定
cap = cv2.VideoCapture(0)  # カメラデバイスIDが0の場合
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 680)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 30)  # フレームレート設定

if not cap.isOpened():
    print("カメラが開けませんでした。")
    exit()

# グローバル変数
running = True
frame = None

# フレーム取得スレッド
def capture_frames():
    global frame, running
    while running:
        ret, new_frame = cap.read()
        if ret:
            frame = new_frame
        else:
            print("フレームが取得できませんでした。")
            break

# フレーム取得スレッドを起動
capture_thread = threading.Thread(target=capture_frames)
capture_thread.start()

# メインループ
try:
    while running:
        if frame is not None:
            # フレームの解像度
            frame_height, frame_width = frame.shape[:2]

            # フレームをRGBに変換
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            # GPU向けデータ型に変換
            frame_tensor = torch.from_numpy(frame_rgb).to(device).float() / 255.0
            frame_tensor = frame_tensor.permute(2, 0, 1).unsqueeze(0)  # (H, W, C) -> (1, C, H, W)

            # YOLOv8で推論
            results = model(frame_tensor, conf=0.5)

            # 検出された物体と距離のリスト
            detected_objects = []

            # 検出されたバウンディングボックスを取得
            for box in results[0].boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])  # バウンディングボックス座標
                conf = box.conf[0]  # 信頼度
                cls = int(box.cls[0])  # クラスID
                Hpixel = y2 - y1  # バウンディングボックスの高さ (ピクセル)

                # 距離を計算
                D = (Hreal * focal_length) / Hpixel

                # 物体の中心座標 (画素)
                cx = (x1 + x2) // 2
                cy = (y1 + y2) // 2

                # カメラの中心座標 (画素)
                frame_center_x = frame.shape[1] // 2
                frame_center_y = frame.shape[0] // 2

                # 中心からの距離を画素単位で計算
                dx_pixels = cx - frame_center_x
                dy_pixels = cy - frame_center_y

                # 実距離（センチメートル）に変換
                dx_cm = (dx_pixels * D) / focal_length
                dy_cm = (dy_pixels * D) / focal_length

                # 検出結果に追加
                detected_objects.append(
                    f"{model.names[cls]}: 距離 {D:.2f} cm, X方向: {dx_cm:.2f} cm, Y方向: {dy_cm:.2f} cm"
                )

                # 描画
                label = f"{model.names[cls]}: {D:.2f} cm, X: {dx_cm:.2f} cm, Y: {dy_cm:.2f} cm"
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # ターミナルに結果を出力
            detected_objects[0] #modelname
            detected_objects[1] #Distance
            detected_objects[2] #dx
            detected_objects[3] #dy
            if detected_objects:
                print("\n検出結果:")
                for obj in detected_objects:
                    print(f" - {obj}")
                if detected_objects[0] == 'alarm':
                    node.dx = detected_objects[1]
                    node.dy = detected_objects[2]
                    node.dd = detected_objects[3]

            else:
                print("\n検出結果: なし")
                node.dx = -1.0
                node.dy = -1.0
                node.dd = -1.0
            #ROS spin
            rclpy.spin_once(node)
            # フレームを表示
            cv2.imshow("Distance Measurement", frame)

        # 'q'キーまたはShift + 'C'で終了
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or (key == ord('c') and cv2.getWindowProperty("Distance Measurement", cv2.WND_PROP_VISIBLE) >= 1):
            running = False
except Exception as e:
    print(f"エラーが発生しました: {e}")

# 終了処理
running = False
capture_thread.join()
cap.release()
cv2.destroyAllWindows()
#ROS2 end
node.destory_node()
rclpy.shutdown()

#motoki ROS2
class YOLONode(Node):
    def __init__(self):
        super().__init__('yolo_node')
        self.pub = self.create_publisher(Vector3, 'yolo', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.dx = 0.0
        self.dy = 0.0
        self.dd = 0.0
    
    def timer_callback(self):
        msg = Vector3()
        msg.x = 0.0
        msg.y = 0.0
        msg.z = 0.0
        self.pub.publish(msg)
        self.get_logger().info(f'Publishing: x={msg.x}, y={msg.y}, D(z)={msg.z}') #debug

