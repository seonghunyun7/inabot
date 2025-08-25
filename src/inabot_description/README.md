# inabot_description

`inabot_description`은 로봇의 URDF(모델 설명), 메시 파일, RViz 설정 및 시뮬레이션 구성을 포함한 패키지입니다.

---
colcon build --packages-select inabot_description
ros2 launch inabot_description inabot_urdf_start.launch.py

## 📦 주요 구성 요소

- **URDF 및 Xacro 파일**  
  실제 및 시뮬레이션용 로봇 모델 정의
- **Mesh 파일 (STL, DAE)**  
  시각적 및 충돌 모델
- **RViz 설정 파일**  
  시각화 및 디버깅을 위한 미리 설정된 RViz 환경
- **Gazebo 월드**  
  테스트 시뮬레이션을 위한 가상 환경 포함

---

📦 전체 TF 트리 구조 (기준: 현재 URDF)
base_link
├── l_wheel
│   └── l_wheel_joint (continuous)
├── r_wheel
│   └── r_wheel_joint (continuous)
├── base_scan
│   └── base_scan_fixed_joint (fixed)
└── imu_link
    └── imu_link_fixed_joint (fixed)

📍 각 링크 설명 및 위치 요약
| 링크 이름       | 부모         | 조인트 종류     | 위치 (XYZ)         | 설명           |
| ----------- | ---------- | ---------- | ---------------- | ------------ |
| `base_link` | -          | -          | (0, 0, 0)        | 로봇의 중심       |
| `l_wheel`   | base\_link | continuous | (0, +0.246, 0)   | 왼쪽 바퀴        |
| `r_wheel`   | base\_link | continuous | (0, -0.246, 0)   | 오른쪽 바퀴       |
| `base_scan` | base\_link | fixed      | (0, 0, 0.15448)  | 라이다 또는 스캔 센서 |
| `imu_link`  | base\_link | fixed      | (0.14, 0, 0.118) | IMU 센서       |

🧭 TF 기준 및 방향
    base_link: 모든 센서 및 휠의 기준 프레임
    좌우 바퀴 (l_wheel, r_wheel):
        y축을 기준으로 좌우 배치됨
        회전축은 y 방향 (axis xyz="0 1 0"), 이는 바퀴가 x방향으로 굴러간다는 의미

    base_scan:
        base_link에서 위쪽으로 약 15.4cm 위치
        스캐너나 라이다 장착용 프레임

    imu_link:
        전방으로 14cm, 위로 11.8cm 위치
        표준 이름 imu_link 사용 중
        IMU는 로봇 중앙 또는 약간 전방에 배치됨


✅ ROS 좌표계 규칙 (REP 103)
    X (빨강): 전방 (forward)
    Y (녹색): 좌측 (left)
    Z (파랑): 위쪽 (up)


| Link Name  | X (m) | Y (m)  | Z (m)   |
| ---------- | ----- | ------ | ------- |
| base\_link | 0.0   | 0.0    | 0.0     |
| l\_wheel   | 0.0   | 0.246  | 0.0     |
| r\_wheel   | 0.0   | -0.246 | 0.0     |
| base\_scan | 0.0   | 0.0    | 0.15448 |
| imu\_link  | 0.14  | 0.0    | 0.118   |


✅ 실제 사용되는 TF 트리 구조

odom (외부에서 제공됨 - SLAM, odometry 등)
 └── base_link (로봇의 중심 프레임)
      ├── base_scan (LiDAR, 고정)
      └── imu_link (IMU, 고정)

🧩 TF 연결 관계 요약
부모 프레임	자식 프레임	용도 / 센서	설명
odom	base_link	로봇 전체 기준	외부 노드에서 제공됨 (예: SLAM)
base_link	base_scan	LiDAR	로봇 중앙 위쪽에 고정
base_link	imu_link	IMU	전방 약간 앞쪽 (0.14m, z=0.118)


base_link
 └── base_scan (fixed, xyz=0 0 0.15448)
      ├── base_scan_front (xyz=0.373455,0.27346,0.106, rpy=0 0 0.767945)
      └── base_scan_rear  (xyz=-0.373455,-0.27346,0.106, rpy=0 0 -2.3911)