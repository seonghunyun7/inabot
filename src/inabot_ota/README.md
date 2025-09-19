서버 : t$ ./ota_test_sender.py for test
  │ 
     <- ota_update.tar.gz 
  |
  ▼
OTA Node (ROS2)
  │
  ├─> 동작 중 노드 종료
  │
  ├─> 기존 install 폴더 → backup 폴더로 이동
  │
  ├─> 압축 해제 → install_new
  │
  ├─> 무결성 체크
  │
  └─> sudo reboot


cd ~/inabot_ws/
colcon build --packages-select inabot_ota
source install/setup.bash

# OTA 노드 실행
ros2 launch inabot_ota ota_launch.py

# OTA 상태 확인
ros2 topic echo /ota_status


# 현재 경로에서 install 폴더를 그대로 압축
tar -czvf ota_update.tar.gz -C ~/inabot_ws install

--

🔹 권장 OTA 흐름
클라이언트 접속 수락
accept() 후 소켓 저장
서버 측으로 "Connection received" 전송
기존 임시 파일 제거

/home/ysh/tmp/ota_update.tar.gz가 존재하면 삭제
상태 메시지 "Previous OTA file removed" 전송

파일 수신
4KB 단위로 읽고 TEMP_ARCHIVE_에 기록
완료 후 상태 메시지 "OTA archive received: <파일경로> (<크기> bytes)" 전송
파일 무결성 확인 (선택)
파일 크기 비교 또는 해시 체크 가능
실패 시 상태 전송 후 중단

업데이트 배포 (deployUpdate())
단계별로 상태 전송

압축 해제
기존 install 백업
새 install 배포

각 단계에서 sendStatusToClient("...") 호출
완료 시 "OTA update completed. Reboot required." 전송

서버 소켓 정리
deployUpdate() 완료 후에도 필요하면 연결 종료
반복적으로 클라이언트 대기 가능

name과 executable의 차이
executable: 실제 실행 파일 이름 → 프로세스 리스트에서 확인 가능 (ps aux | grep <executable>).
name: ROS 노드 이름 → ROS2 내부에서만 쓰임, 프로세스 이름과는 다를 수 있음.
→ 단, 노드 이름과 executable 이름이 같으면 둘 다 가능.

## tar -czvf 
c → 새 아카이브 생성 (create)
z → gzip 압축 적용
v → 진행 상황 표시 (verbose, 생략 가능)
f → 파일 이름 지정 (ota_update.tar.gz)

tar -czvf ota_update.tar.gz install
