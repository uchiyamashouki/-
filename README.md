# sazaesann
このパッケージは、CRANE-X7とRealsenseD435を使用し、人の手とじゃんけんをする
ROS2パッケージです。

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

## このパッケージを使う前に
### ROS2及びCRANE-X7セットアップ
- ROS2インストール
  上田先生の[動画](https://www.youtube.com/watch?v=mBhtD08f5KY)及び[インストールスクリプト](https://github.com/ryuichiueda/ros2_setup_scripts)を参照し、インストールを行ってください。
  
- CRANE-X7及び関連パッケージのインストール
  [RT社リポジトリ](https://github.com/rt-net/crane_x7_ros/tree/ros2)よりインストールできます。[RT社のブログ](https://rt-net.jp/humanoid/archives/4653)でも解説されています。いかにインストールコマンドを載せます。
  ```
  # Setup ROS environment
  source /opt/ros/humble/setup.bash
  
  # Download crane_x7 repositories
  mkdir -p ~/ros2_ws/src
  cd ~/ros2_ws/src
  git clone -b ros2 https://github.com/rt-net/crane_x7_ros.git
  git clone -b ros2 https://github.com/rt-net/crane_x7_description.git
  
  # Install dependencies
  rosdep install -r -y -i --from-paths .

  # Build & Install
  cd ~/ros2_ws
  colcon build --symlink-install
  source ~/ros2_ws/install/setup.bash
  ```
  ([RT社リポジトリREADME.md](https://github.com/rt-net/crane_x7_ros/blob/ros2/README.md#installation)より転載)
  
  また, インストールが完了したらパッケージに含まれるサンプルコードをシミュレータ（Gazebo）で試すことができます。詳しくは[こちら](https://github.com/rt-net/crane_x7_ros/tree/ros2/crane_x7_examples)を参照してください。
  
- USBポートの設定（実機のCRANE-X7を動かす際に必要となります）
  ```
  # 一時的な付与の場合(上手くいかない時はUSBポートの名前を確認してください)
  sudo chmod 666 /dev/ttyUSB0
  
  # 永続的な付与の場合(再起動を伴います)
  sudo usermod -aG dialout $USER
  reboot
  ```

### RealSenseセットアップ
[IntelRealsenseのgithub](https://github.com/IntelRealSense/librealsense/blob/development/doc/distribution_linux.md#installing-the-packages)を参照してください。以下先ほどのページより引用。
```
# Register the server's public key:
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null

# Make sure apt HTTPS support is installed:
sudo apt-get install apt-transport-https

# Add the server to the list of repositories:
echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
sudo tee /etc/apt/sources.list.d/librealsense.list
sudo apt-get update

# Install the libraries (see section below if upgrading packages):
sudo apt-get install librealsense2-dkms
sudo apt-get install librealsense2-utils

# セットアップできているか確認（ビューワーの起動）:  
realsense-viewer
```
# このパッケージの使い方
## インストール
```
cd  ~/ros2_ws/src
https://github.com/uchiyamashouki/sazaesann.git
```
## ビルド
```
cd ~/ros2_ws
colcon build
source ~/ros2_ws/install/setup.bash

# ３行目のコマンドは、~/.bashrcに書いておくことを推奨します.   
# 下のコマンドで.bashrcに追記できます.  
echo 'source ~/ros2_ws/install/setup.bash' >> ~/.bachrc
# .bashrcに書いてあるとき下のコマンドが代わりにできます.
source ~/.bashrc
```

## 実行
```
#RealSense D435を搭載したcrane-x7のmove_groupと controllerの起動
$ ros2 launch crane_x7_examples demo.launch.py port_name:=/dev/ttyUSB0 use_d435:=true

# 本パッケージの実行
$ ros2 launch sazaesann sazaesann.launch.py 
```
- 実行したらカメラの前でじゃんけんの手をそれぞれ出し、キーボード操作G,T,P,でその手を保存する。
キーボードと手は下記の様に対応する。
    - 「グー」はG,
    - 「チョキ」はT,
    - 「パー」P,

- じゃんけんの手を3秒出し続けるとその手を確定し、その手を元にロボットが動き出す。
- 手が確定してから5秒後に再度手を認識するようになる。

## 動作確認済み環境
Ubuntu 22.04

## ライセンス
- © 2024 cit_sazae24
- 本パッケージはApache License, Version 2.0に基づき公開されています。  
ライセンスの全文は[LICENSE](./LICENSE)または[https://www.apache.org/licenses/LICENSE-2.0](https://www.apache.org/licenses/LICENSE-2.0)から確認できます。
