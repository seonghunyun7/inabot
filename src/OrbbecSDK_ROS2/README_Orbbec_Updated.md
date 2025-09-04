# Orbbec Camera ROS2 Integration (Updated)

Ubuntu 22.04 기준으로 Orbbec SDK 설치, ROS2 빌드 및 카메라 노드 실행, QR 코드(with opencv qr api) 및 IMU 처리 예제

---

## Orbbec SDK 설치

###  SDK 설치 패키지 다운로드
https://orbbec.github.io/OrbbecSDK_v2/
https://github.com/orbbec/OrbbecSDK_v2/releases

cd ~/Downloads
sudo dpkg -i OrbbecSDK_v2.4.11_amd64.deb
sudo apt-get install -f


###  Udev rules 설정 및 의존 성 패키지 설치
./setup_orbbec_ros2.sh 


### 3. 설치 확인
OrbbecViewer_v2.4.11_202508040937_a6f3117_linux_x86_64.zip
orbbec_viewer


---

## ROS2 git 소스 다운 및  빌드
cd inabot_ws/
cd src/
git clone https://github.com/orbbec/OrbbecSDK_ROS2.git
cd inabot_ws/
colcon build --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

---

## 카메라 노드 실행

### 터미널 1: 카메라 노드 실행
cd ~/ros2_ws/
source install/setup.bash
ros2 launch orbbec_camera astra.launch.py
```
---

### 터미널 2: RViz 실행
cd ~/ros2_ws/
source install/setup.bash
rviz2
```
---

### 터미널 3: 사용 가능한 토픽 확인
ros2 topic list
ros2 service list
ros2 param list
```

---

## 주요 데이터 토픽

### IMU Data
- `/camera/accel/imu_info`
- `/camera/gyro/imu_info`
- `/camera/gyro_accel/sample`

### Color Camera
- `/camera/color/camera_info`
- `/camera/color/image_raw`
- `/camera/color/image_raw/compressed`
- `/camera/color/image_raw/compressedDepth`
- `/camera/color/image_raw/theora`
- `/camera/color/metadata`

### Depth Camera
- `/camera/depth/camera_info`
- `/camera/depth/image_raw`
- `/camera/depth/image_raw/compressed`
- `/camera/depth/image_raw/compressedDepth`
- `/camera/depth/image_raw/theora`
- `/camera/depth/metadata`
- `/camera/depth/points`
- `/camera/depth_filter_status`
- `/camera/depth_registered/points`
- `/camera/depth_to_color`
- `/camera/depth_to_left_ir`
- `/camera/depth_to_right_ir`

### Infrared Camera
- `/camera/left_ir/camera_info`
- `/camera/left_ir/image_raw`
- `/camera/left_ir/image_raw/compressed`
- `/camera/left_ir/image_raw/compressedDepth`
- `/camera/left_ir/image_raw/theora`
- `/camera/left_ir/metadata`
- `/camera/right_ir/camera_info`
- `/camera/right_ir/image_raw`
- `/camera/right_ir/image_raw/compressed`
- `/camera/right_ir/image_raw/compressedDepth`
- `/camera/right_ir/image_raw/theora`
- `/camera/right_ir/metadata`

### Miscellaneous
- `/diagnostics`
- `/parameter_events`
- `/rosout`
- `/rosout_agg`

---

## Python Examples

### 5.1 컬러 영상 구독 + OpenCV 처리
python3 camera_processor.py

### 5.2 QR 코드 인식 (컬러 영상 기준)
qr_code_detector.py

### 5.3 QR + Depth + IMU 처리 예제
qr_imu_processor.py
