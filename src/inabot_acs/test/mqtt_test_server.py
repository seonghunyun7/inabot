import paho.mqtt.client as mqtt
import time
import json

# 토픽별 핸들러 정의
def handle_timesync(payload):
    print(f"[TimeSync] Payload: {payload}")

def handle_heartbeat(payload):
    print(f"[Heartbeat] Payload: {payload}")

def handle_ack(payload):
    print(f"[Ack] Payload: {payload}")

def handle_mission(payload):
    print(f"[Mission] Payload: {payload}")

def handle_control(payload):
    print(f"[Control] Payload: {payload}")

# 토픽별 핸들러 매핑
topic_handlers = {
    "timesync": handle_timesync,
    "heartbeat": handle_heartbeat,
    "ack": handle_ack,
    "mission": handle_mission,
    "control": handle_control,
}

def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    # 토픽 구독 시작
    for topic in topic_handlers.keys():
        client.subscribe(topic)
    client.publish("timesync", json.dumps({"msg": "Hello from Python MQTT server!"}))

def on_message(client, userdata, msg):
    payload = msg.payload.decode()
    print(f"Received on {msg.topic}: {payload}")

    handler = topic_handlers.get(msg.topic)
    if handler:
        try:
            json_data = json.loads(payload)
            handler(json_data)
        except json.JSONDecodeError:
            print(f"Invalid JSON on topic {msg.topic}")
    else:
        print(f"No handler for topic {msg.topic}")

# MQTT Client 설정
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.connect("localhost", 1883, 60)
client.loop_start()

# 예제 데이터 생성 함수
def generate_timesync():
    return {
        "seq": 1,
        "time": time.time(),
        "start_time": time.time() - 5
    }

def generate_heartbeat():
    return {
        "seq": 2,
        "time": time.time()
    }

def generate_ack():
    return {
        "seq": 3,
        "time": time.time(),
        "command_name": "StartMission",
        "command_seq": 101,
        "result": 1
    }

def generate_mission_move():
    return {
        "seq": 5,
        "time": time.time(),
        "idx": 7,
        "waypoint": {
            "position": {
                "x": 1.1,
                "y": 2.2,
                "z": 0.0
            },
            "orientation": {
                "x": 0.0,
                "y": 0.0,
                "z": 1.0,
                "w": 0.0
            }
        },
        "speed": 0.3,
        "head_basis": 1,
        "head_angle": 0.0,
        "strict_move": False,
        "docking": False,
        "amr_heading": 45,
        "direction": 0,
        "diagonal_curve": 0.0,
        "landmark_id": ""
    }

def generate_mission_action():
    return {
        "seq": 6,
        "time": time.time(),
        "idx": 2,
        "command": "Wait",
        "trigger": 5.0,
        "hold_position": True,
        "param": [-1, 0, 0, 0, 0, 0, 0]
    }

def generate_mission():
    return {
        "seq": 4,
        "time": time.time(),
        "mission_id": 12,
        "move": [generate_mission_move()],
        "action": []
    }

def generate_control():
    return {
        "seq": 7,
        "time": time.time(),
        "command": "Stop"
    }

# 메인 루프
try:
    while True:
        client.publish("timesync", json.dumps(generate_timesync()))
        client.publish("heartbeat", json.dumps(generate_heartbeat()))
        client.publish("ack", json.dumps(generate_ack()))
        client.publish("mission", json.dumps(generate_mission()))
        client.publish("control", json.dumps(generate_control()))
        time.sleep(5)
except KeyboardInterrupt:
    client.loop_stop()
    client.disconnect()
