# class_agnostic_object_detector_ros

## 使用方法

Humble @ Ubuntu 22.04 @ Jetson AGX Orin でテスト済み

### 本体のクローン

```bash
cd ~ # 例
git clone git@github.com:mitukou1109/mvits_for_class_agnostic_od.git -b python3.10
```

### 仮想環境の作成

```bash
cd ~/mvits_for_class_agnostic_od
virtualenv -p python3 .venv # 仮想環境マネージャは何でもいいがシステムインタプリタを使うこと
source .venv/bin/activate
```

### PyTorch のインストール

> AMD64 の場合は`pip install torch torchvision torchaudio`で OK

Jetson の場合

```bash
wget raw.githubusercontent.com/pytorch/pytorch/5c6af2b583709f6176898c017424dc9981023c28/.ci/docker/common/install_cusparselt.sh
```

install_cusparselt.sh の 7 行目あたりに`CUDA_VERSION=12.4`を追記

```bash
sudo bash install_cusparselt.sh
pip install http://jetson.webredirect.org/jp6/cu126/+f/5cf/9ed17e35cb752/torch-2.5.0-cp310-cp310-linux_aarch64.whl#sha256=5cf9ed17e35cb7523812aeda9e7d6353c437048c5a6df1dc6617650333049092
pip install http://jetson.webredirect.org/jp6/cu126/+f/5f9/67f920de3953f/torchvision-0.20.0-cp310-cp310-linux_aarch64.whl#sha256=5f967f920de3953f2a39d95154b1feffd5ccc06b4589e51540dc070021a9adb9
```

### 依存パッケージをインストール

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
RUSTFLAGS="-A invalid_reference_casting" pip install -r requirements.txt
cd models/ops
./make.sh
pip install opencv-python
deactivate
```

### ROS 2 ラッパーをクローン・ビルド

```bash
cd ~/minitruck_ws/src # 例
git clone git@github.com:mitukou1109/class_agnostic_object_detector_ros.git
cd ..
colcon build --cmake-args -DPython3_FIND_VIRTUALENV=STANDARD # symlink-installするとimportエラーになる（原因不明）
```

### 実行

カメラ（[`usb_cam`](https://github.com/ros-drivers/usb_cam)、[`v4l2_camera`](https://gitlab.com/boldhearts/ros2_v4l2_camera)）や画像ファイル（[`image_publisher`](https://github.com/ros-perception/image_pipeline/tree/humble/image_publisher)）等の入力ソースを用意

```bash
source ~/minitruck_ws/install/local_setup.bash
source ~/mvits_for_class_agnostic_od/.venv/bin/activate
PYTHONPATH=$PYTHONPATH:$HOME:$HOME/mvits_for_class_agnostic_od ros2 run class_agnostic_object_detector_ros object_detector_node --ros-args -p checkpoint_file:=/path/to/checkpoint/file
```
