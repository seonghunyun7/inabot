Factory 패턴 + SOLID 원칙



✳️ 디렉토리 기준 역할 정리
디렉토리	역할 요약
motor/	실제 모터 드라이브 동작 (motor.cpp)
motor_controller/	모터 초기화 전용 추상화 및 생성 (Factory 패턴)
motor_kinematics/	모터 속도 ↔ 바퀴/로봇 움직임 변환
can_interface	CAN 프레임 송수신
imu/, odom/, monitor/	주변 센서 및 상태 감시
utils/	공용 도구: 로그, 예외 등
wheel/	바퀴 그룹 로직
kinematics/	전체 구동 계열 제어 (mecanum, omni 등)
🧩 향후 확장 예
기능	추가 파일
Elmo 드라이버	elmo_motor_controller.hpp/.cpp
모터 자동 초기화	yaml-cpp와 연동해 factory 호출
Mock 테스트용	mock_motor_controller.hpp/.cpp
유닛 테스트	test_motor_controller.cpp (gtest 등으로 별도 작성)