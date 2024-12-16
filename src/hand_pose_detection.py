#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
import numpy as np
import time


# カメラ選択    True: Realsense, False: PC,
USE_REALSENSE = True


# 相対座標に変換
def landmark2np(hand_landmarks):
    li = []
    for j in hand_landmarks.landmark:
        li.append([j.x, j.y, j.z])
    return np.array(li) - li[0]


# cos類似度
def manual_cos(A, B):
    dot = np.sum(A * B, axis=-1)
    A_norm = np.linalg.norm(A, axis=-1)
    B_norm = np.linalg.norm(B, axis=-1)
    cos = dot / (A_norm * B_norm + 1e-7)
    return cos[1:].mean()


class HandPosePublisher(Node):
    def __init__(self):
        super().__init__('hand_pose_publisher')

        self.bridge = CvBridge()
        self.publisher_ = self.create_publisher(String, '/hand_pose', 10)

	#カメラ設定
        if USE_REALSENSE:
            # Realsenseの場合はトピックから購読
            self.image_subscription = self.create_subscription(
                Image,
                '/camera/color/image_raw',
                self.image_callback,
                10
            )
        else:
            self.cap = cv2.VideoCapture(0)

        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands()
        self.mp_draw = mp.solutions.drawing_utils

        self.saved_array = [None, None, None]
        self.start = -100
        self.score = [0, 0, 0]
        self.saved_no = 0
        self.previous_pose = None
        self.previous_pose_time = None
        self.last_published_time = 0

        self.pose_similarity = 0.99             # ポーズ判定の類似度
        self.pose_hold_duration = 2.0           # ポーズを維持する時間
        self.publish_cooldown = 5.0             # クールダウンの時間

    def process_frame_pc(self):
        """パソコンのカメラのフレームを処理"""
        _, img = self.cap.read()
        self.process_frame(img)

    def image_callback(self, msg):
        """Realsenseのフレームを処理"""
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.process_frame(img)
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

    def process_frame(self, img):
        """フレームの共通処理"""
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        results = self.hands.process(imgRGB)

        key = cv2.waitKey(1) & 0xFF
        current_pose = None  # 現在のポーズ

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                # 手のランドマークを描画
                for i, lm in enumerate(hand_landmarks.landmark):
                    height, width, _ = img.shape
                    cx, cy = int(lm.x * width), int(lm.y * height)
                    cv2.putText(img, str(i + 1), (cx + 10, cy + 10), cv2.FONT_HERSHEY_PLAIN, 4, (255, 255, 255), 5, cv2.LINE_AA)
                    cv2.circle(img, (cx, cy), 10, (255, 0, 255), cv2.FILLED)
                self.mp_draw.draw_landmarks(img, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)

                # キーボードGTPでポーズを保存
                if key == ord('g'):
                    self.saved_array[0] = landmark2np(hand_landmarks)
                    self.start = time.time()
                    self.saved_no = 1
                    self.get_logger().info('guu saved')

                if key == ord('t'):
                    self.saved_array[1] = landmark2np(hand_landmarks)
                    self.start = time.time()
                    self.saved_no = 2
                    self.get_logger().info('tyoki saved')

                if key == ord('p'):
                    self.saved_array[2] = landmark2np(hand_landmarks)
                    self.start = time.time()
                    self.saved_no = 3
                    self.get_logger().info('paaaa saved')

                # ポーズを計算
                now_array = landmark2np(hand_landmarks)
                for i in range(3):
                    if self.saved_array[i] is not None:
                        self.score[i] = manual_cos(self.saved_array[i], now_array)

                # ポーズ認識の条件判定
                if self.score[0] > self.pose_similarity:
                    current_pose = 'guu'
                elif self.score[1] > self.pose_similarity:
                    current_pose = 'tyoki'
                elif self.score[2] > self.pose_similarity:
                    current_pose = 'paaaa'

        # クールダウン終了かつ、 ポーズを維持されたらパブリッシュ
        if time.time() - self.last_published_time >= self.publish_cooldown:
            if current_pose == self.previous_pose:
                if self.previous_pose_time is None:
                    self.previous_pose_time = time.time()

                if current_pose and time.time() - self.previous_pose_time >= self.pose_hold_duration:
                    self.get_logger().info(f'Published: {current_pose}')
                    self.publisher_.publish(String(data=current_pose))
                    self.last_published_time = time.time()
                    self.previous_pose_time = None
            else:
                # ポーズが変わった場合はリセット
                self.previous_pose = current_pose
                self.previous_pose_time = time.time() if current_pose else None
        else:
            cv2.putText(img, 'published', (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # カメラ映像を表示
        if current_pose:
            cv2.putText(img, current_pose, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 2)
        cv2.imshow("Image", img)


def main(args=None):
    rclpy.init(args=args)
    node = HandPosePublisher()
    if USE_REALSENSE:
        rclpy.spin(node)
    else:
        while rclpy.ok():
            node.process_frame_pc()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

