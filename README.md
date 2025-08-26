# Inabot Project Structure

이 저장소는 ROS2 기반 로봇 플랫폼 **Inabot**의 전체 소스 코드입니다.  
각 폴더/패키지의 역할은 다음과 같습니다.

---

## 주요 패키지

### `src/inabot_core/`
로봇 핵심 기능 모듈
- 센서 (IMU, Odom)
- 모터 드라이버 및 제어기
- 주행 제어 알고리즘 (MPC, Pure Pursuit 등)
- 경로 계획 (A*, Global Planner 등)
- 상태 모니터링

### `src/inabot_description/`
로봇 모델 정의
- URDF/Xacro
- Mesh(시각/충돌 모델)
- RViz 설정, Gazebo world

### `src/inabot_interface/`
시스템 전체 인터페이스 다이어그램 등 문서 자료

### `src/inabot_msgs/`
커스텀 메시지, 서비스, 액션 정의

### `src/inabot_slam/`
SLAM 관련 패키지
- Cartographer 기반 2D 맵핑/로컬라이제이션 설정
- Launch/Config 파일

### `src/inabot_teleop_joy/`
조이스틱/키보드 원격 제어
- `inabot_joy`: ROS2 joy 기반 조이스틱 제어
- `inabot_teleop`: 키보드 원격 제어

---

## 기타 디렉토리

### `python/`
보조 스크립트 및 테스트 도구
- Dummy path generator
- Trajectory plotter
- Motor load plotter
- Pose graph

### `robot_upstart/`
로봇 서비스를 systemd에 등록하기 위한 설정
- 서비스 자동 시작/중지 스크립트
- 템플릿 및 설치 스크립트

### `setup/`
ROS2 환경 설치 및 의존성 스크립트

--- 

 

