# sound_driver

`ROS2 Humble` 환경에서 USB 또는 내장 오디오 장치를 사용하여 로봇 사운드를 재생하는 C++ 노드 패키지입니다.

---

## 📦 패키지 구조

sound_driver/
├── CMakeLists.txt
├── package.xml
├── include/
│ └── sound.hpp
├── src/
│ └── sound.cpp
└── launch/
└── sound_driver.launch.py

- **include/sound.hpp** : Sound 클래스 헤더
- **src/sound.cpp** : Sound 노드 구현
- **launch/sound_driver.launch.py** : Sound 노드 단독 실행 런처

---

## 🔧 설치 및 빌드

cd ~/inabot_ws/
colcon build --packages-select sound_driver
source install/setup.bash
 
예: run.mp3, obs.mp3, err.mp3, stop.mp3

▶️ 실행
ros2 launch sound_driver sound_driver.launch.py

실행 시 노드 이름: sound_node

ROS2 토픽:
    /sound/play (std_msgs/String) : 재생할 사운드 타입 (default, obstacle, error, stop)
    /sound/status (std_msgs/String) : 재생 상태 퍼블리시
    /sound/device_status (std_msgs/Bool) : USB 오디오 장치 상태

사용 예제
ros2 topic pub /sound/play std_msgs/String "data: 'default'"
ros2 topic pub /sound/play std_msgs/String "data: 'obstacle'"
ros2 topic pub /sound/play std_msgs/String "data: 'error'"

'default' : 기본 소리 (run.mp3)
'obstacle' : 장애물 경고 소리 (obs.mp3)
'error' : 에러 알림 (err.mp3)
'stop' : 재생 중지

사운드 재생
ros2 topic pub /robot_sound_volume std_msgs/Float64 "data: 0.8"


⚙️ 주의사항
USB 오디오 장치가 연결되어 있어야 합니다. (check_sound_card_usb_audio() 자동 감지)
ffplay 설치 필요: sudo apt install ffmpeg