# いつでもサザエ  個人
crane-x7とROS2を用いたじゃんけんプロジェクト
個人での開発用

## 依存
- [ROS2 Humble](https://github.com/ros2)
- [crane_x7_ros](https://github.com/rt-net/crane_x7_ros/tree/ros2)
- [IntelRealSense](https://github.com/IntelRealSense)
- [mediapipe](https://github.com/google-ai-edge/mediapipe)
- [opencv-python](https://github.com/opencv/opencv-python)
- [scikit-learn](https://github.com/scikit-learn/scikit-learn)

## ソースファイル
- color_detection.cpp
  - 選択した色の座標を計算
- pick_and_move_tf.cpp
  - 色の座標を元にロボットを操作
- color_selector.cpp
  - 手のポーズを元に色を選択
- hand_pose_detection.py
  - 手のポーズを読み取り
  

## インストール
関連パッケージをインストール

本パッケージを```~/ros2_ws/src/```にダウンロード、ビルド

## 実行
```
#RealSense D435を搭載したcrane-x7のmove_groupと controllerの起動
$ ros2 launch crane_x7_examples demo.launch.py port_name:=/dev/ttyUSB0 use_d435:=true

# 本パッケージの実行
$ ros2 launch itudemo_sazae itudemo_sazae.launch.py 
```

## ライセンス
- © 2024 tentoshinz
- 本パッケージはApache License, Version 2.0に基づき公開されています。  
ライセンスの全文は[LICENSE](./LICENSE)または[https://www.apache.org/licenses/LICENSE-2.0](https://www.apache.org/licenses/LICENSE-2.0)から確認できます。
