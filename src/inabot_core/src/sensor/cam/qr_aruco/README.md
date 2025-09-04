
개요

본 시스템은 RGB 카메라와 IMU 데이터를 사용하여 **대차(운반 카트)**의 위치와 방향을 실시간으로 인식하고, 이를 기반으로 로봇이 대차 아래로 정확히 진입할 수 있도록 경로를 생성합니다.

ysh@ysh-ThinkPad-T16-Gen-4:~/temps$ python3 -c "import cv2; print(cv2.__version__)"
4.5.4

참조 코드 : 아루코
   https://github.com/JMU-ROBOTICS-VIVA/ros2_aruco

주요 흐름:

1. 카메라로 마커 인식

   ArUco 마커 또는 QR 코드를 카메라 영상에서 인식

   OpenCV ArUco 모듈(cv::aruco) 및 QR 디코더(cv::QRCodeDetector) 사용

   마커 ID와 영상 상의 코너 좌표 추출

2. 대차 좌표 계산

   카메라 내부 파라미터와 마커 크기(marker_size)를 이용해 3D 위치 추정

   IMU 데이터를 활용하여 카메라 기준 좌표를 로봇 좌표로 변환

   결과: 대차 중심 좌표 및 회전 각도(yaw)

3. 경로 생성

   대차 위치와 방향에 기반하여 로봇 이동 경로 계획

   로봇이 대차 아래로 안전하게 진입하도록 경로 생성

4. ROS 토픽 발행

   /aruco_poses (geometry_msgs/PoseArray): 모든 인식 마커의 Pose

   /aruco_markers (inabot_msgs/ArucoMarkers): Pose + 마커 ID

## 구현 방식
   | 구분       | 사용 기술                                                                    |
   | -------- | ------------------------------------------------------------------------ |
   | 마커 인식    | OpenCV ArUco (`cv::aruco::detectMarkers`) / QR 코드 (`cv::QRCodeDetector`) |
   | 카메라 파라미터 | ROS `sensor_msgs/CameraInfo`                                             |
   | IMU 보정   | ROS `sensor_msgs/Imu` / yaw 기준 좌표 변환                                     |
   | 3D 위치 추정 | `cv::aruco::estimatePoseSingleMarkers`                                   |
   | ROS 퍼블리싱 | C++ ROS2 Node (`rclcpp`)                                                 |

## 마커 설정

   마커 크기: 5 cm (marker_size: 0.05)

   Aruco 딕셔너리: DICT_5X5_250 (QR 마커는 cv::QRCodeDetector)

   카메라 해상도: 최소 640x480 권장

   인식 거리: 1~2 m 내에서 안정적 인식


## 순서도

   [RGB 카메라 이미지 수신] 
           |
           v
[마커 인식]
  - ArUco: cv::aruco::detectMarkers
  - QR: cv::QRCodeDetector
           |
           v
[3D 좌표 계산]
  - 카메라 내부 파라미터 활용
  - IMU 기반 로봇 좌표 변환
           |
           v
[대차 위치/방향 추출]
  - 중심 좌표
  - yaw 각도
           |
           v
[경로 생성]
  - 대차 아래로 진입 경로 계산
           |
           v
[로봇 이동 제어]
           |
           v
[ROS 토픽 발행]
  - /aruco_poses (PoseArray)
  - /aruco_markers (Pose + ID)


| Dictionary Name       | int 값 | 설명                |
| --------------------- | ----- | ----------------- |
| DICT\_4X4\_50         | 0     | 4×4 비트, 50개 마커    |
| DICT\_4X4\_100        | 1     | 4×4 비트, 100개 마커   |
| DICT\_4X4\_250        | 2     | 4×4 비트, 250개 마커   |
| DICT\_4X4\_1000       | 3     | 4×4 비트, 1000개 마커  |
| DICT\_5X5\_50         | 4     | 5×5 비트, 50개 마커    |
| DICT\_5X5\_100        | 5     | 5×5 비트, 100개 마커   |
| DICT\_5X5\_250        | 6     | 5×5 비트, 250개 마커   |
| DICT\_5X5\_1000       | 7     | 5×5 비트, 1000개 마커  |
| DICT\_6X6\_50         | 8     | 6×6 비트, 50개 마커    |
| DICT\_6X6\_100        | 9     | 6×6 비트, 100개 마커   |
| DICT\_6X6\_250        | 10    | 6×6 비트, 250개 마커   |
| DICT\_6X6\_1000       | 11    | 6×6 비트, 1000개 마커  |
| DICT\_7X7\_50         | 12    | 7×7 비트, 50개 마커    |
| DICT\_7X7\_100        | 13    | 7×7 비트, 100개 마커   |
| DICT\_7X7\_250        | 14    | 7×7 비트, 250개 마커   |
| DICT\_7X7\_1000       | 15    | 7×7 비트, 1000개 마커  |
| DICT\_ARUCO\_ORIGINAL | 16    | 초기 ArUco 라이브러리 마커 |
| DICT\_APRILTAG\_16h5  | 17    | AprilTag 16h5     |
| DICT\_APRILTAG\_25h9  | 18    | AprilTag 25h9     |
| DICT\_APRILTAG\_36h10 | 19    | AprilTag 36h10    |
| DICT\_APRILTAG\_36h11 | 20    | AprilTag 36h11    |


aruco_id = 마커를 고유하게 식별하는 번호

Dictionary마다 ID 범위가 정해져 있음 (예: DICT_5X5_250 → 0~249)

카메라가 마커를 감지하면 ID를 반환 → 로봇 위치 인식, 물체 인식, 트리거 등 다양한 용도로 사용



QR 마커 인지
 
# 소스 다운로드
cd ~/ros_ws/src
git clone https://github.com/nu-book/zxing-cpp.git


# 빌드
# 의존성 설치
sudo apt update
sudo apt install git cmake g++ libpng-dev

# 소스 다운로드
cd ~/src
git clone --recursive https://github.com/nu-book/zxing-cpp.git
cd zxing-cpp

# 빌드
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON ..
make -j$(nproc)
sudo make install

ysh@ysh-ThinkPad-T16-Gen-4:/usr/local/lib$ 

libZXing.so.2.3.0
libZXing.so.3 -> libZXing.so.2.3.0

Consolidate compiler generated dependencies of target ZXingOpenCV
[100%] Built target ZXingOpenCV
Install the project...
-- Install configuration: "Release"
-- Installing: /usr/local/lib/libZXing.so.2.3.0
-- Installing: /usr/local/lib/libZXing.so.3
-- Installing: /usr/local/lib/libZXing.so
-- Installing: /usr/local/include/ZXing/Barcode.h
-- Installing: /usr/local/include/ZXing/BarcodeFormat.h
-- Installing: /usr/local/include/ZXing/BitHacks.h
-- Installing: /usr/local/include/ZXing/ByteArray.h
-- Installing: /usr/local/include/ZXing/CharacterSet.h
-- Installing: /usr/local/include/ZXing/Content.h
-- Installing: /usr/local/include/ZXing/Error.h
-- Installing: /usr/local/include/ZXing/Flags.h
-- Installing: /usr/local/include/ZXing/GTIN.h
-- Installing: /usr/local/include/ZXing/ImageView.h
-- Installing: /usr/local/include/ZXing/Point.h
-- Installing: /usr/local/include/ZXing/Quadrilateral.h
-- Installing: /usr/local/include/ZXing/Range.h
-- Installing: /usr/local/include/ZXing/ReadBarcode.h
-- Installing: /usr/local/include/ZXing/ReaderOptions.h
-- Installing: /usr/local/include/ZXing/StructuredAppend.h
-- Installing: /usr/local/include/ZXing/TextUtfEncoding.h
-- Installing: /usr/local/include/ZXing/ZXingCpp.h
-- Installing: /usr/local/include/ZXing/ZXAlgorithms.h
-- Installing: /usr/local/include/ZXing/ZXVersion.h
-- Installing: /usr/local/include/ZXing/DecodeHints.h
-- Installing: /usr/local/include/ZXing/Result.h
-- Installing: /usr/local/include/ZXing/BitMatrix.h
-- Installing: /usr/local/include/ZXing/BitMatrixIO.h
-- Installing: /usr/local/include/ZXing/Matrix.h
-- Installing: /usr/local/include/ZXing/MultiFormatWriter.h
-- Installing: /usr/local/include/ZXing/Version.h
-- Installing: /usr/local/lib/cmake/ZXing/ZXingTargets.cmake
-- Installing: /usr/local/lib/cmake/ZXing/ZXingTargets-release.cmake
-- Installing: /usr/local/lib/pkgconfig/zxing.pc
-- Installing: /usr/local/lib/cmake/ZXing/ZXingConfig.cmake
-- Installing: /usr/local/lib/cmake/ZXing/ZXingConfigVersion.cmake
-- Installing: /usr/local/bin/ZXingWriter
-- Set runtime path of "/usr/local/bin/ZXingWriter" to ""
-- Installing: /usr/local/bin/ZXingReader
-- Set runtime path of "/usr/local/bin/ZXingReader" to ""


==

[카메라 이미지 수신]
        ↓
 [카메라 정보 수신?]
     ├─ 아니오 → 대기 (return)
     └─ 예 → 진행
        ↓
 [cv_bridge 변환 → Gray 변환]
        ↓
 ┌───────────── QR 처리 ───────────────┐
 │ #if OpenCV_QR → OpenCV QRCodeDetector│
 │ #else → ZXing ReaderOptions(QRCode) │
 │ QR 코드 검출 성공 시 로그 + 시각화 │
 └─────────────────────────────────────┘
        ↓
 ┌───────────── ArUco 처리 ─────────────┐
 │ cv::aruco::detectMarkers             │
 │ → 코너 검출 + ID 반환                │
 │ Pose 계산 (cv::aruco::estimatePose..)│
 │ → Rotation + Translation 벡터 추출   │
 │ → Quaternion 변환(tf2::Matrix3x3)    │
 │ PoseArray + Marker 메시지 생성       │
 └─────────────────────────────────────┘
        ↓
[검출 결과 Publish]
 - PoseArray
 - ArUcoMarkers
 - 디버그 이미지 (QR + ArUco 표시)
        ↓
[종료/대기]



| 항목    | **QR 코드**                 | **ArUco 마커**              |
| ----- | ------------------------- | ------------------------- |
| 목적    | 데이터 저장 (문자열, URL, 숫자 등)   | ID 식별 (마커 인식/위치 추정)       |
| 스펙    | ISO/IEC 18004 국제 표준       | OpenCV 등에서 정의된 Dictionary |
| 크기    | 21×21 \~ 177×177 (버전에 따라) | 보통 4×4, 5×5, 6×6, 7×7     |
| 에러 보정 | 최대 30% 손상 복구              | 해밍 거리 기반 ID 인식 (간단)       |
| 사용 예  | URL, 제품 정보, 스마트폰 스캔       | 로봇 위치 인식, AR/VR 트래킹       |
