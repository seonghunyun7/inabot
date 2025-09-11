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
 

## VDA5050 프로토콜 2.0 기준의 MQTT 토픽 구조이고, FMS 서버(혹은 Master Control PC)와 AGV 간의 통신 방향

| Subtopic                 | 설명                 |
| ------------------ | ------------------ |
| `"order"`          | AGV가 수행할 작업 지시 메시지 |
| `"instantActions"` | 즉시 실행해야 하는 액션 명령   |
| `"state"`          | AGV 상태 보고          |
| `"visualization"`  | 시각화용 정보            |
| `"connection"`     | 연결 정보              |
| `"factsheet"`      | 시스템 정보             |


| Subtopic           | Published by   | Subscribed by         | 방향                  | 용도                  | Schema                  |
| ------------------ | -------------- | --------------------- | ------------------- | ------------------- | ----------------------- |
| **order**          | Master Control | AGV                   | PC → AGV            | 주행 명령 전송            | `order.schema`          |
| **instantActions** | Master Control | AGV                   | PC → AGV            | 즉시 수행할 동작 전송        | `instantActions.schema` |
| **state**          | AGV            | Master Control        | AGV → PC            | AGV 상태 통신           | `state.schema`          |
| **visualization**  | AGV            | Visualization Systems | AGV → Visualization | 고빈도 위치 정보, 시각화용     | `visualization.schema`  |
| **connection**     | Broker/AGV     | Master Control        | AGV → PC            | 연결 상태 표시              | `connection.schema`     |
| **factsheet**      | AGV            | Master Control        | AGV → PC            | AGV 초기 설정 정보 전달     | `factsheet.schema`      |


6.14 Topic "connection"
During the connection of an AGV client to the broker, a last will topic and message can be
set, which is published by the broker upon disconnection of the AGV client from the broker.
Thus, the master control can detect a disconnection event by subscribing the connection
topics of all AGV. 

// VDA5050 2.0 표준에서 AGV와 MQTT 브로커 간의 연결 상태를 주기적으로 확인하고 관리하는 방식이 정의

The disconnection is detected via a ""heartbeat"" that is exchanged between
the broker and the client. The interval is configurable in most brokers and should be set
around 15 seconds. The Quality of Service level for the connection topic shall be 1 - At
Least Once.
The suggested last will topic structure is:
uagv/v2/manufacturer/SN/connection

==> MQTT Last Will & Testament (LWT) 기능 활용
AGV가 예기치 않게 연결이 끊겼을 때 브로커가 자동으로 "connection" 토픽에 상태 메시지를 발행
하비트 역할
브로커와 클라이언트 간의 연결이 살아 있는지 확인
대부분 브로커에서 하비트 간격(keepalive interval) 설정 가능 (권장 15초)
AGV → 브로커, 브로커 → Master Control 간 연결 모니터링
 ==> VDA5050 2.0 표준에서는 connection 토픽이 하비트(heartbeat)의 역할을 수행하며, 
    AGV는 주기적인 publish 대신 MQTT keepalive와 LWT를 활용

## 
AGV가 MQTT 브로커에 연결될 때 Last Will를 설정 → 연결 끊기 시 브로커가 자동으로 CONNECTIONBROKEN 발행
AGV가 정상 종료 시 → "OFFLINE" 상태 발행 후 disconnect
연결 중 → "ONLINE" 상태 발행
메시지는 retained flag 설정 → 최신 상태 유지
QoS = 1 (At least once)
ROS2 노드 내부에서 주기적 메시지 발행은 필요 없음 (MQTT keepalive 활용)

| 구분        | 사용 용도                  | 시각 포함 여부    |
| --------- | ---------------------- | ----------- |
| LWT       | 예기치 않은 연결 종료 시 브로커가 발행 | 고정값, 실시간 아님 |
| Heartbeat | 주기적 상태 모니터링            | 현재 시각 포함 가능 |

[Robot] ---> (MQTT CONNECT) ---> [Broker]
   |                               |
   |                               |
   |-------> 비정상 종료 ---------> LWT 발행 ----> (Subscribed clients, 예: FMS)


로봇이 끊긴 순간 브로커가 LWT 토픽으로 "OFFLINE" 메시지 발행
구독하고 있는 FMS나 다른 클라이언트가 메시지를 받음

=> LWT 메시지는 로봇이 직접 보내는 것이 아니라, 브로커가 로봇 연결이 비정상적으로 끊겼을 때 설정된 토픽으로 발행

1. Mosquitto (터미널 0)
sudo systemctl start mosquitto

2. LWT (터미널 1)
mosquitto_sub -h localhost -t "uagv/v2/Inatech/P3LDD02/connection" -v

3. ros2 launch inabot_acs robot_acs_launch.py (터미널 3); LWT 설정 포함 (lwt_enabled=True, lwt_topic=..., lwt_payload=...)

4. 노드 강제 종료 (터미널 3)
ps aux | grep robot_acs_node
kill -9 17835

(1) 프로세스 강제 종료
kill -9 <pid>로 노드 강제 종료
LWT 메시지 확인 : 미널 1에서 LWT 메시지가 자동으로 출력되는지 확인

네트워크 끊기
로컬 환경에서 브로커와 클라이언트 연결을 끊어도 가능
sudo ifconfig lo down   # 로컬 연결 끊기
sudo ifconfig lo up     # 복구

-------------------------------------------------------------------------

. vd5050d order(topic) => drop

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

# . vd5050d order(topic) => move 
mosquitto_pub -h localhost -p 1883 -t "order" -m '{
  "headerId": 8096,
  "timestamp": "2025-06-30T04:52:11.39Z",
  "version": "2.0.0",
  "manufacturer": "Inatech",
  "serialNumber": "P3LDD02",
  "orderId": "ACS-1000122678",
  "orderUpdateId": 0,
  "zoneSetId": "",
  "nodes": [
    {
      "nodeId": "10002",
      "sequenceId": 0,
      "released": true,
      "nodeDescription": "",
      "nodePosition": {
        "x": 221.33,
        "y": -44.428,
        "mapId": "P3",
        "allowedDeviationXY": 0.5
      },
      "actions": []
    },
    {
      "nodeId": "89",
      "sequenceId": 2,
      "released": true,
      "nodeDescription": "",
      "nodePosition": {
        "x": 222.7,
        "y": -44.428,
        "mapId": "P3",
        "allowedDeviationXY": 0.2
      },
      "actions": []
    }
  ],
  "edges": [
    {
      "edgeId": "10002_89",
      "sequenceId": 1,
      "released": true,
      "startNodeId": "10002",
      "endNodeId": "89",
      "maxSpeed": 0.05,
      "orientation": 3.141592653589793,
      "rotationAllowed": false,
      "actions": []
    }
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


--

    1. Heartbeat 개념 구분
        ◦ 브로커 차원의 heartbeat는 MQTT 표준 keepalive 메커니즘에 의해 관리됩니다.
클라이언트가 정해진 주기 내에 패킷을 송신하지 않으면 브로커가 연결이 끊어진 것으로 간주하고, 설정된 LWT(Last Will & Testament) 메시지를 발행합니다.
        ◦ 반면, VDA5050 사양의 heartbeat 메시지는 애플리케이션 레벨에서 AGV가 주기적으로 state 메시지를 publish하도록 정의한 것입니다.
이를 통해 FMS는 단순 네트워크 연결 상태뿐 아니라, AGV가 정상적으로 동작하고 있음을 주기적으로 확인할 수 있습니다.
        ◦ 따라서 두 메커니즘은 상호 보완적으로 동작하며,
            ▪ 브로커 keepalive/LWT는 네트워크 연결 상태를 보장하고,
            ▪ VDA5050 heartbeat는 애플리케이션 레벨의 AGV 생존 상태를 보장합니다.
    2. Docker 환경 vs Mosquitto 브로커 환경
        ◦ 기본적으로 동일한 MQTT 표준을 따르므로 heartbeat 동작 자체에는 차이가 없습니다.
        ◦ 다만 Docker 환경에서는 컨테이너 네트워크 설정과 리소스 제한이 변수로 작용할 수 있고,
Mosquitto 단독 환경에서는 OS 네트워크 설정, 브로커 파라미터(keepalive, max_inflight_messages 등)에 영향을 받을 수 있습니다.
    3. 현장 대응 가이드 (예정)
        ◦ P4H 현장에서 대응 인원들이 직접 확인할 수 있도록, 다음 절차를 권장합니다.
            ▪ MQTT 모니터링 툴(예: mosquitto_sub, MQTT Explorer)로 heartbeat/state 토픽 구독
            ▪ 정해진 주기(예: 15초 이내)로 메시지가 수신되는지 확인
            ▪ 연결 해제 시 LWT 토픽(connection)이 CONNECTIONBROKEN으로 변하는지 확인
        ◦ 이를 기반으로 내부 대응 매뉴얼을 준비할 예정입니다.
    4. P3D 운영 구간 통신 불안정
        ◦ 네트워크 품질 문제(와이파이 핸드오버, AP 로밍 지연 등)로 간헐적 끊김이 발생할 수 있습니다.
        ◦ 대응 방안으로는 브로커 keepalive 최적화, QoS=1 설정, 재연결 파라미터 조정, 네트워크 로밍 최적화 등을 고려하실 수 있으며, 필요 시 저희 설정값을 공유드릴 수 있습니다.

정리
    • 브로커 heartbeat: MQTT 표준 keepalive/LWT → 네트워크 연결 상태 확인
    • VDA5050 heartbeat: AGV state 주기적 publish → 애플리케이션 레벨 상태 확인
    • 두 메커니즘을 함께 사용해야 안정적인 FMS–AGV 통신 상태 보장이 가능합니다.
