제안하는 구조 및 클래스 설계

include/
└── monitor/
    ├── monitor.hpp          // 추상 기본 모니터 클래스 (옵션)
    ├── device_topic_monitor.hpp   // 센서 상태 모니터링 
    ├── io_monitor.hpp       // IO 디바이스 상태 모니터링
    ├── network_monitor.hpp  // 네트워크 상태 모니터링 (이미 있음)
    └── system_monitor.hpp   // 시스템 상태 모니터링 (이미 있음)
    
src/
└── monitor/
    ├── device_topic_monitor.cpp
    ├── io_monitor.cpp
    ├── network_monitor.cpp
    └── system_monitor.cpp

역할 분담 예시

    DeviceTopicMonitor

        위치/속도	odom	nav_msgs/msg/Odometry
        배터리 상태	bms_status	lrbot2_msgs/msg/BmsStatus
        엔코더	/encoder	sensor_msgs/msg/JointState
        IMU	/imu	sensor_msgs/msg/Imu
        라이다 (앞/뒤/합성)	scan_front, scan_rear, scan_merged

        추가로 고려할 수 있는 토픽
        1. TF (Transform) 상태
            tf (또는 /tf, /tf_static)
            타입: tf2_msgs/msg/TFMessage
            용도: 센서 간 위치 관계가 올바르게 설정되어 있는지 확인
            ✔ 필요하면 transform 브로드캐스터 작동 여부도 확인 가능

        2. Diagnostics
            /diagnostics
            타입: diagnostic_msgs/msg/DiagnosticArray
            용도: 하드웨어/소프트웨어 진단 상태. 센서/모터/CPU 온도 등 포함 가능
            ✔ ROS 1 기반 로봇에서는 필수적이었고, ROS 2에서도 사용 가능

        3. Battery (표준 메시지)
            /battery_state (혹은 /power_status)
            타입: sensor_msgs/msg/BatteryState
            용도: 표준 ROS 메시지를 사용하는 배터리 장치가 있는 경우 유용

        4. CPU / 메모리 사용량 (노드 단위)
            /resource_monitor, /node_status, /system_status
            커스텀 메시지 또는 diagnostic_msgs
            용도: 노드 단위 상태 파악 (죽었는지, 느려졌는지 등)

        5. 충돌 감지 / 긴급 정지
            /emergency_stop, /collision_detected, /bumper_state
            타입: std_msgs/Bool, 커스텀 메시지
            용도: 로봇이 충돌했거나 긴급 정지 신호가 들어온 경우 알림

        6. 컨트롤 명령
            /cmd_vel
            타입: geometry_msgs/msg/Twist
            용도: 실제로 로봇이 움직이라는 명령이 제대로 들어오는지 확인
            ✔ 로봇이 안 움직일 때 cmd_vel이 안 들어오는지 확인하는 데도 유용

        7. 로컬라이제이션 상태
            예: /amcl_pose, /initialpose, /map
            위치 추정 성능, 초기화 여부 확인

        8. Navigation 상태 (Nav2 등 사용 시)
            /goal_pose, /nav_status, /plan, /local_costmap
            네비게이션 상태, 목적지 도달 여부, 플래너 동작 등을 확인

        9. 모터 제어 상태 (확장 가능 시)
            /motor_feedback, /motor_cmd
            모터 응답 속도, 제어 명령이 들어왔는지 여부 등

    IoMonitor

        외부 디바이스 가령 PLC ..등 IO 디바이스 상태 점검

        장치 연결 여부 및 통신 상태 확인

        이상 징후 발견 시 경고 발신

    NetworkMonitor

        이미 있음, 네트워크 연결 상태 모니터링

    SystemMonitor

        CPU, 메모리, 디스크, 프로세스 상태 등 시스템 전반 상태 체크
 