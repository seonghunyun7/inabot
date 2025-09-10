# inabot_acs README

## 프로젝트 개요
`inabot_acs` 프로젝트는 FMS(Fleet Management System)와 ROS 2 간의 브릿지 역할을 수행합니다.  
MQTT를 통해 명령 및 상태 정보를 주고받으며, ROS 2 토픽을 통해 로봇 제어와 상태 피드백을 처리합니다.

**통신 흐름:**

- **FMS → ROS 2**: MQTT 명령 메시지 전송
- **ROS 2 → FMS**: ROS 2 토픽 구독 후 MQTT 상태 메시지 전송

---

## 프로젝트 구조

inabot_acs/
├── CMakeLists.txt
├── include/
│ ├── inabot_acs/
│ │ ├── acs_node.hpp
│ │ ├── message_handler.hpp
│ │ ├── mission_manager.hpp
│ │ └── mqtt_client.hpp
│ └── json/single_include/nlohmann/json.hpp
├── launch/
│ └── inabot_acs.launch.py
├── src/
│ ├── main.cpp -> ROS 2 노드 초기화 및 실행
│ ├── acs_node.cpp -> AcsNode 클래스 구현, MissionManager 및 MQTT Client 초기화
│ ├── message_handler.cpp -> 메시지 처리 로직 구현
│ ├── mission_manager.cpp -> 미션 관리 및 ROS 2 토픽 발행 구현
│ └── mqtt_client.cpp -> MQTT Client 통신 구현
├── test/
│ └── mqtt_test_server.py
└── README.md

---

## ROS 2 MQTT Client Node

**기능:**

- FMS에서 MQTT 명령 메시지 수신 → ROS 2 토픽 publish
- ROS 2 토픽에서 로봇 상태/결과 수신 → MQTT로 FMS에 publish

**주요 파라미터:**

| 파라미터 | 설명 | 기본값 |
|-----------|------|--------|
| `broker_host` | MQTT 브로커 주소 | localhost |
| `broker_port` | MQTT 브로커 포트 | 1883 |
| `tls_enabled` | TLS 사용 여부 | false |
| `client_id` | MQTT 클라이언트 ID | fms_client |
| `clean_session` | 세션 저장 여부 | true |
| `keep_alive_interval` | Ping 메시지 간격(초) | 60 |
| `max_inflight` | 최대 메시지 수 | 65535 |

---

## 빌드 방법 (ROS 2 Humble 기준)

```bash
# ROS 2 환경 설정
source /opt/ros/humble/setup.bash
sudo apt install libfmt-dev

# 워크스페이스 빌드
cd ~/robot_ws
colcon build --symlink-install --packages-select inabot_acs
or 
colcon build --packages-select inabot_acs

ROS 2 노드 실행

ros2 launch inabot_acs inabot_acs.launch.py

MQTT 라이브러리 설치
1. Paho MQTT C 라이브러리

git clone https://github.com/eclipse/paho.mqtt.c.git
cd paho.mqtt.c
mkdir build && cd build
cmake ..
make
sudo make install

설치 경로:

    라이브러리: /usr/local/lib/libpaho-mqtt3as.so

    헤더: /usr/local/include/

2. Paho MQTT C++ 라이브러리

git clone https://github.com/eclipse/paho.mqtt.cpp.git
cd paho.mqtt.cpp
git submodule update --init --recursive
mkdir build && cd build
cmake .. -DPAHO_WITH_MQTT_C=ON
make
sudo make install

설치 경로:

    라이브러리: /usr/local/lib/libpaho-mqttpp3.so

    헤더: /usr/local/include/

    설치 경로가 다를 경우 CMAKE_PREFIX_PATH 또는 CMAKE_INCLUDE_PATH를 설정하세요:

export CMAKE_PREFIX_PATH=/usr/local

통신 테스트
1. Mosquitto 브로커 설치 및 실행

sudo apt install mosquitto mosquitto-clients
sudo systemctl start mosquitto
sudo ufw allow 1883   # 방화벽 포트 열기

2. 테스트용 MQTT 퍼블리셔/서브스크라이버

# MQTT 테스트 서버 (Python)
python3 ~/robot_ws/src/inabot_acs/test/mqtt_test_server.py

3. ROS 2 노드 실행

ros2 launch inabot_acs inabot_acs.launch.py

4. 테스트 확인

    fms/command 토픽 → ROS 2에서 수신

    robot/status 토픽 → MQTT로 FMS 전송

    QoS 0,1,2 확인 가능

 
         +-------------------+
         |  MQTT Broker (서버) |
         |    (Mosquitto)     |
         +---------+---------+
                   |
    +--------------+---------------+
    |                              |
+---+---+                     +----+----+
| Python | (Publisher+Sub)     |  C++    | (Subscriber)
| Client |                     | Client  |
+-------+                     +---------+
 
1). MQTT 브로커 띄우기 (서버)
    - 로컬 터미널(1)에 Mosquitto 설치/ localhost에 MQTT 브로커 실행

sudo apt install mosquitto (미 설치 시)
sudo systemctl start mosquitto
sudo systemctl status mosquitto <- 동작 확인

2) Client 실행 - 로컬 터미널(2)
ros2 launch inabot_acs inabot_acs.launch.py

3) 테스트용 MQTT 퍼블리셔/서브스크라이버 사용 (터미널 3)

sudo apt update
sudo apt install mosquitto-clients

4) 주의 사항
1883 포트 열기
MQTT 브로커가 사용하는 1883 포트를 열려면 다음 명령어를 입력하세요:
sudo ufw allow 1883

포트를 열고 방화벽 규칙을 다시 확인하려면:
sudo ufw status
 


VDA5050+사양서.doc
| 토픽                 | 설명                 |
| ------------------ | ------------------ |
| `"order"`          | AGV가 수행할 작업 지시 메시지 |
| `"instantActions"` | 즉시 실행해야 하는 액션 명령   |
| `"state"`          | AGV 상태 보고          |
| `"visualization"`  | 시각화용 정보            |
| `"connection"`     | 연결 정보              |
| `"factsheet"`      | 시스템 정보             |

. vd5050d order(topic) > drop

mosquitto_pub -h localhost -p 1883 -t "order" -m '{
  "headerId": 8669,
  "timestamp": "2025-07-04T07:32:05.50Z",
  "version": "2.0.0",
  "manufacturer": "Inatech",
  "serialNumber": "P3LDD02",
  "orderId": "ACS-1000122799",
  "orderUpdateId": 0,
  "zoneSetId": "",
  "nodes": [
    {
      "nodeId": "10121",
      "sequenceId": 0,
      "released": true,
      "nodeDescription": "",
      "nodePosition": {"x": 2.97723E2,"y": 6.1E0,"mapId": "P3","allowedDeviationXY": 2E-1},
      "actions": []
    },
    {
      "nodeId": "10122",
      "sequenceId": 2,
      "released": true,
      "nodeDescription": "",
      "nodePosition": {"x": 2.9825E2,"y": 6.1E0,"mapId": "P3","allowedDeviationXY": 1E-1},
      "actions": []
    },
    {
      "nodeId": "10123",
      "sequenceId": 4,
      "released": true,
      "nodeDescription": "",
      "nodePosition": {"x": 2.985E2,"y": 6.1E0,"mapId": "P3","allowedDeviationXY": 2E-1},
      "actions": []
    },
    {
      "nodeId": "10124",
      "sequenceId": 6,
      "released": true,
      "nodeDescription": "",
      "nodePosition": {"x": 2.999E2,"y": 6.1E0,"mapId": "P3","allowedDeviationXY": 5E-1},
      "actions": [
        {"actionId": "drop","actionParameters": [], "actionType": "drop","blockingType": "HARD"}
      ]
    }
  ],
  "edges": [
    {"edgeId": "10121_10122","sequenceId": 1,"released": true,"startNodeId": "10121","endNodeId": "10122","maxSpeed": 1E-1,"orientation": 3.141592653589793E0,"rotationAllowed": false,"actions":[]},
    {"edgeId": "10122_10123","sequenceId": 3,"released": true,"startNodeId": "10122","endNodeId": "10123","maxSpeed": 1E-1,"orientation": 3.141592653589793E0,"rotationAllowed": false,"actions":[]},
    {"edgeId": "10123_10124","sequenceId": 5,"released": true,"startNodeId": "10123","endNodeId": "10124","maxSpeed": 1E-1,"orientation": 3.141592653589793E0,"rotationAllowed": false,"actions":[]}
  ]
}'


## for_test


a. heartbeat 보내기

mosquitto_pub -h localhost -p 1883 -t "heartbeat" -m '{
  "seq":2,
  "time":1714814500
}'

b. mission 보내기

mosquitto_pub -h localhost -p 1883 -t "mission" -m '{
  "seq":4,
  "time":1714814500,
  "mission_id":12,
  "move":[
    {
      "seq":5,
      "time":1714814500,
      "idx":7,
      "waypoint":{
        "position":{"x":1.1,"y":2.2,"z":0.0},
        "orientation":{"x":0.0,"y":0.0,"z":1.0,"w":0.0}
      },
      "speed":0.3,
      "head_basis":1,
      "head_angle":0.0,
      "strict_move":false,
      "docking":false,
      "amr_heading":45,
      "direction":0,
      "diagonal_curve":0.0,
      "landmark_id":""
    }
  ],
  "action":[]
}'

c. mission - action 

mosquitto_pub -h localhost -p 1883 -t "mission" -m '{
  "seq":6,
  "time":1714814500,
  "mission_id":13,
  "move":[],
  "action":[
    {
      "seq":7,
      "time":1714814501,
      "idx":2,
      "command":"Wait",
      "trigger":5.0,
      "hold_position":true,
      "param":[-1, 0, 0, 0, 0, 0, 0]
    }
  ]
}'

6. control 보내기

mosquitto_pub -h localhost -p 1883 -t "control" -m '{
  "seq":7,
  "time":1714814500,
  "command":"Stop"
}'

7. map request

mosquitto_pub -h localhost -p 1883 -t "map" -m '{
  "seq": 123,
  "time": 456789,
  "command": "ReqMap",
  "layer": 0,
  "version": 0,
  "payload": {}
}'


## mosquitto
sudo systemctl status mosquitto

부팅 후에도 자동 실행되게	sudo systemctl enable mosquitto
부팅 후 자동 실행 안되게	sudo systemctl disable mosquitto
지금 mosquitto를 실행	sudo systemctl start mosquitto
지금 mosquitto를 중지	sudo systemctl stop mosquitto
systemctl is-enabled mosquitto


# 안전하게 테스트하는 방법

이전 로봇 노드 프로세스 종료
ps aux | grep robot_acs_node

ysh@ysh-ThinkPad-T16-Gen-4:~/inabot_ws$ ps aux | grep robot_acs_node
ysh        29798  0.0  0.0  10844  2560 pts/1    R+   17:12   0:00 grep --color=auto robot_acs_node

kill <PID>
