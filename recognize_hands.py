#!/usr/bin/python3
import cv2
import mediapipe as mp
import time
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray
from collections import deque

def main():
    # 初期座標を指定
    WRIST_x = 300
    WRIST_y = 200
    INDEX_FINGER_TIP_y = 200
    MIDDLE_FINGER_TIP_y = 200
    RING_FINGER_TIP_y = 200
    PINKY_FINGER_TIP_y = 200
    INDEX_FINGER_TIP_x = 200
    MIDDLE_FINGER_TIP_x = 200
    RING_FINGER_TIP_x = 200
    PINKY_FINGER_TIP_x = 200
    INDEX_FINGER_MCP_y = 300
    INDEX_FINGER_MCP_x = 300
    MIDDLE_FINGER_MCP_x = 300
    RING_FINGER_MCP_x = 300
    PINKY_FINGER_MCP_x = 300
    fx = 300*3.2
    fy = 200*2.7
    D_INDEX = 20000
    D_MIDDLE = 20000
    D_RING = 20000
    D_PINKY = 20000
    DST = 120
    D_HUND = 100
    clamp_position = 0

    # 平滑化のためのパラメータ
    SMOOTHING_WINDOW_SIZE = 5

    # 座標保存用のデータ構造
    wrists_x = deque(maxlen=SMOOTHING_WINDOW_SIZE)
    wrists_y = deque(maxlen=SMOOTHING_WINDOW_SIZE)

    index_finger_tip_x = deque(maxlen=SMOOTHING_WINDOW_SIZE)
    index_finger_tip_y = deque(maxlen=SMOOTHING_WINDOW_SIZE)

    middle_finger_tip_x = deque(maxlen=SMOOTHING_WINDOW_SIZE)
    middle_finger_tip_y = deque(maxlen=SMOOTHING_WINDOW_SIZE)

    ring_finger_tip_x = deque(maxlen=SMOOTHING_WINDOW_SIZE)
    ring_finger_tip_y = deque(maxlen=SMOOTHING_WINDOW_SIZE)

    pinky_finger_tip_x = deque(maxlen=SMOOTHING_WINDOW_SIZE)
    pinky_finger_tip_y = deque(maxlen=SMOOTHING_WINDOW_SIZE)

    index_finger_mcp_x = deque(maxlen=SMOOTHING_WINDOW_SIZE)
    index_finger_mcp_y = deque(maxlen=SMOOTHING_WINDOW_SIZE)

    middle_finger_mcp_x = deque(maxlen=SMOOTHING_WINDOW_SIZE)
    middle_finger_mcp_y = deque(maxlen=SMOOTHING_WINDOW_SIZE)

    ring_finger_mcp_x = deque(maxlen=SMOOTHING_WINDOW_SIZE)
    ring_finger_mcp_y = deque(maxlen=SMOOTHING_WINDOW_SIZE)

    pinky_finger_mcp_x = deque(maxlen=SMOOTHING_WINDOW_SIZE)
    pinky_finger_mcp_y = deque(maxlen=SMOOTHING_WINDOW_SIZE)

    rclpy.init()
    node = Node("MediaPipe_node")
    qos_profile = QoSProfile(depth=10)
    hand_position = node.create_publisher(Float32MultiArray, 'hand_topic', qos_profile)

    # Webカメラから入力
    cap = cv2.VideoCapture(0)

    print("\x1b[2J") 

    # Handモデルのインスタンス化
    with mp.solutions.hands.Hands(
        model_complexity=0,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5) as hands:

        # ウィンドウ表示の無限ループ
        while True:

            success, image = cap.read()
            if not success:
                print("Ignoring empty camera frame.")
                smoothed_WRIST_x, smoothed_WRIST_y, result_degrees, position, clamp_position = 0
                array_points = Float32MultiArray()
                array_points.data = [smoothed_WRIST_x, smoothed_WRIST_y, result_degrees, position, clamp_position]  
                hand_position.publish(array_points) 
                continue
            image.flags.writeable = False
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            
            # 手の位置検出と描画
            results = hands.process(image)
            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    for index, landmark in enumerate(hand_landmarks.landmark):
                        # 関節の座標を取得
                        x, y = int(landmark.x * image.shape[1]), int(landmark.y * image.shape[0])

                        # 各指の先の位置を取得
                        if index == 8:  # インデックス指の先
                            INDEX_FINGER_TIP_y = y
                            INDEX_FINGER_TIP_x = x
                        elif index == 12:  # 中指の先
                            MIDDLE_FINGER_TIP_y = y
                            MIDDLE_FINGER_TIP_x = x
                        elif index == 16:  # 薬指の先
                            RING_FINGER_TIP_y = y
                            RING_FINGER_TIP_x = x
                        elif index == 20:  # 小指の先
                            PINKY_FINGER_TIP_y = y
                            PINKY_FINGER_TIP_x = x
                        elif index == 5:  # 中指の先
                            INDEX_FINGER_MCP_y = y
                            INDEX_FINGER_MCP_x = x
                        elif index == 9:  # 薬指の先
                            MIDDLE_FINGER_MCP_x = x
                            MIDDLE_FINGER_MCP_y = y
                        elif index == 13:  # 小指の先
                            RING_FINGER_MCP_x = x
                            RING_FINGER_MCP_y = y
                        elif index == 17:  # 小指の先
                            PINKY_FINGER_MCP_x = x
                            PINKY_FINGER_MCP_y = y
                        elif index == 4:  
                            THUMB_FINGER_TIP_x = x
                            THUMB_FINGER_TIP_y = y

                        #print(f"Hand {results.multi_handedness[0].classification[0].label}: Joint {index} - X: {x}, Y: {y}")

                    # 検出された手の骨格をカメラ画像に重ねて描画
                    mp.solutions.drawing_utils.draw_landmarks(
                        image,
                        hand_landmarks,
                        mp.solutions.hands.HAND_CONNECTIONS)

            # カメラ画像を表示
            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            cv2.imshow('MediaPipe Hands', cv2.flip(image, 1))

            if  results.multi_hand_landmarks:
                wrist_landmark = results.multi_hand_landmarks[0].landmark[0]
                WRIST_x, WRIST_y = int(wrist_landmark.x * image.shape[1]), int(wrist_landmark.y * image.shape[0])

                # 最新の座標をリストに追加
                wrists_x.append(WRIST_x)
                wrists_y.append(WRIST_y)

                index_finger_tip_x.append(INDEX_FINGER_TIP_x)
                index_finger_tip_y.append(INDEX_FINGER_TIP_y)

                middle_finger_tip_x.append(MIDDLE_FINGER_TIP_x)
                middle_finger_tip_y.append(MIDDLE_FINGER_TIP_y)

                ring_finger_tip_x.append(RING_FINGER_TIP_x)
                ring_finger_tip_y.append(RING_FINGER_TIP_y)

                pinky_finger_tip_x.append(PINKY_FINGER_TIP_x)
                pinky_finger_tip_y.append(PINKY_FINGER_TIP_y)

                index_finger_mcp_x.append(INDEX_FINGER_MCP_x)
                index_finger_mcp_y.append(INDEX_FINGER_MCP_y)

                middle_finger_mcp_x.append(MIDDLE_FINGER_MCP_x)
                middle_finger_mcp_y.append(MIDDLE_FINGER_MCP_y)

                ring_finger_mcp_x.append(RING_FINGER_MCP_x)
                ring_finger_mcp_y.append(RING_FINGER_MCP_y)

                pinky_finger_mcp_x.append(PINKY_FINGER_MCP_x)
                pinky_finger_mcp_y.append(PINKY_FINGER_MCP_y)

                # 平滑化された座標を計算
                smoothed_WRIST_x = sum(wrists_x) / len(wrists_x)
                smoothed_WRIST_y = sum(wrists_y) / len(wrists_y)

                smoothed_INDEX_FINGER_TIP_x = sum(index_finger_tip_x) / len(index_finger_tip_x)
                smoothed_INDEX_FINGER_TIP_y = sum(index_finger_tip_y) / len(index_finger_tip_y)

                smoothed_MIDDLE_FINGER_TIP_x = sum(middle_finger_tip_x) / len(middle_finger_tip_x)
                smoothed_MIDDLE_FINGER_TIP_y = sum(middle_finger_tip_y) / len(middle_finger_tip_y)

                smoothed_RING_FINGER_TIP_x = sum(ring_finger_tip_x) / len(ring_finger_tip_x)
                smoothed_RING_FINGER_TIP_y = sum(ring_finger_tip_y) / len(ring_finger_tip_y)

                smoothed_PINKY_FINGER_TIP_x = sum(pinky_finger_tip_x) / len(pinky_finger_tip_x)
                smoothed_PINKY_FINGER_TIP_y = sum(pinky_finger_tip_y) / len(pinky_finger_tip_y)

                smoothed_INDEX_FINGER_MCP_x = sum(index_finger_mcp_x) / len(index_finger_mcp_x)
                smoothed_INDEX_FINGER_MCP_y = sum(index_finger_mcp_y) / len(index_finger_mcp_y)

                smoothed_MIDDLE_FINGER_MCP_x = sum(middle_finger_mcp_x) / len(middle_finger_mcp_x)
                smoothed_MIDDLE_FINGER_MCP_y = sum(middle_finger_mcp_y) / len(middle_finger_mcp_y)

                smoothed_RING_FINGER_MCP_x = sum(ring_finger_mcp_x) / len(ring_finger_mcp_x)
                smoothed_RING_FINGER_MCP_y = sum(ring_finger_mcp_y) / len(ring_finger_mcp_y)

                smoothed_PINKY_FINGER_MCP_x = sum(pinky_finger_mcp_x) / len(pinky_finger_mcp_x)
                smoothed_PINKY_FINGER_MCP_y = sum(pinky_finger_mcp_y) / len(pinky_finger_mcp_y)


                # 三平方の定理を使用して、平滑化された距離を計算
                D_INDEX = math.sqrt((smoothed_WRIST_y - smoothed_INDEX_FINGER_TIP_y)**2 + (smoothed_WRIST_x - smoothed_INDEX_FINGER_TIP_x)**2)
                D_MIDDLE = math.sqrt((smoothed_WRIST_y - smoothed_MIDDLE_FINGER_TIP_y)**2 + (smoothed_WRIST_x - smoothed_MIDDLE_FINGER_TIP_x)**2)
                D_PINKY = math.sqrt((smoothed_WRIST_y - smoothed_PINKY_FINGER_TIP_y)**2 + (smoothed_WRIST_x - smoothed_PINKY_FINGER_TIP_x)**2)
                D_RING = math.sqrt((smoothed_WRIST_y - smoothed_RING_FINGER_TIP_y)**2 + (smoothed_WRIST_x - smoothed_RING_FINGER_TIP_x)**2)
                D_HUND = math.sqrt((smoothed_WRIST_y - smoothed_INDEX_FINGER_MCP_y)**2 + (smoothed_WRIST_x - smoothed_INDEX_FINGER_MCP_x)**2)
                D_PINKY_MCP = math.sqrt((smoothed_WRIST_y - smoothed_PINKY_FINGER_MCP_y)**2 + (smoothed_WRIST_x - smoothed_PINKY_FINGER_MCP_x)**2)
                D_WIDE = math.sqrt((smoothed_INDEX_FINGER_MCP_y - smoothed_PINKY_FINGER_MCP_y)**2 + (smoothed_INDEX_FINGER_MCP_x - smoothed_PINKY_FINGER_MCP_x)**2)

                DST = D_HUND * 1.2

                print("\n\n\n\n\n\n\n\n")
                print("X: ",smoothed_WRIST_x)
                print("Y: ",smoothed_WRIST_y)

                # arctan(1/√3) を計算し、ラジアンで取得
                result_radians = math.asin(math.sqrt((smoothed_WRIST_y - smoothed_PINKY_FINGER_MCP_y)**2) / D_PINKY_MCP)
                # ラジアンを度に変換
                result_degrees = math.degrees(result_radians)

                if smoothed_WRIST_x < smoothed_PINKY_FINGER_MCP_x :
                    if smoothed_WRIST_y >smoothed_PINKY_FINGER_MCP_y :
                        result_degrees = result_degrees + 20
                    else :
                        result_degrees = result_degrees * -1 + 20
                if smoothed_WRIST_x > smoothed_PINKY_FINGER_MCP_x :
                    if smoothed_WRIST_y > smoothed_PINKY_FINGER_MCP_y :
                        result_degrees = 200 - result_degrees 
                    else :
                        result_degrees = 200 + result_degrees 
                print("計算結果（度）:", result_degrees)

                if D_HUND  != 0 and D_WIDE != 0:
                    position = D_HUND/D_WIDE
                print("傾き", position)

                # 手首と指の位置の差の条件を設定して握りこみの判定
                if  (D_RING < DST/1.3)  and (D_PINKY < DST/1.3) :

                    print("clamp  ")
                    clamp_position = 1

                    fx = smoothed_WRIST_x * 3.2
                    fy = (smoothed_WRIST_y - 200) * 2.7
                else :
                    print("release")
                    clamp_position = 0

                array_points = Float32MultiArray()
                array_points.data = [smoothed_WRIST_x, smoothed_WRIST_y, result_degrees, position, clamp_position]  
                hand_position.publish(array_points) 

                print("\x1b[0;0H")  

                D_HUND == 0
            # Escキーで終了
            if cv2.waitKey(5) & 0xFF == 27:
                print("\x1b[2J") 
                break

    # HandモデルとWebカメラの解放とOpenCVウィンドウの破棄
    cap.release()
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
