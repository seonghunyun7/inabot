import time
import json
import paho.mqtt.client as mqtt
from datetime import datetime

# -----------------------------
# VD5050 토픽 정의
# -----------------------------
VDA_INTERFACE = "uagv"
VDA_VERSION = "v2.0.0"
MANUFACTURER = "RobotCompany"
SERIAL_NUMBER = "Robot001"

TOPIC_ORDER = f"{VDA_INTERFACE}/{VDA_VERSION}/{MANUFACTURER}/{SERIAL_NUMBER}/order"
TOPIC_INSTANT_ACTIONS = f"{VDA_INTERFACE}/{VDA_VERSION}/{MANUFACTURER}/{SERIAL_NUMBER}/instantActions"
TOPIC_STATE = f"{VDA_INTERFACE}/{VDA_VERSION}/{MANUFACTURER}/{SERIAL_NUMBER}/state"
TOPIC_CONNECTION = f"{VDA_INTERFACE}/{VDA_VERSION}/{MANUFACTURER}/{SERIAL_NUMBER}/connection"

# -----------------------------
# 토픽별 핸들러
# -----------------------------
def handle_order(payload):
    print(f"[ORDER] Payload: {json.dumps(payload, indent=2)}")

def handle_instant_actions(payload):
    print(f"[INSTANT_ACTIONS] Payload: {json.dumps(payload, indent=2)}")

def handle_state(payload):
    print(f"[STATE] Payload: {json.dumps(payload, indent=2)}")

def handle_connection(payload):
    print(f"[CONNECTION] Payload: {json.dumps(payload, indent=2)}")

topic_handlers = {
    TOPIC_ORDER: handle_order,
    TOPIC_INSTANT_ACTIONS: handle_instant_actions,
    TOPIC_STATE: handle_state,
    TOPIC_CONNECTION: handle_connection,
}

# -----------------------------
# MQTT 이벤트
# -----------------------------
def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    for topic in topic_handlers.keys():
        client.subscribe(topic)
    print("[INFO] Subscribed to VD5050 topics")

def on_message(client, userdata, msg):
    payload = msg.payload.decode()
    handler = topic_handlers.get(msg.topic)
    if handler:
        try:
            json_data = json.loads(payload)
            handler(json_data)
        except json.JSONDecodeError:
            print(f"Invalid JSON on topic {msg.topic}")

# -----------------------------
# MQTT 클라이언트 초기화
# -----------------------------
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect("localhost", 1883, 60)
client.loop_start()

# -----------------------------
# 메시지 생성 함수
# -----------------------------
def generate_order():
    timestamp = datetime.utcnow().strftime("%Y-%m-%dT%H:%M:%SZ")
    return {
        "headerId": 1,
        "timestamp": timestamp,
        "version": "2.0.0",
        "manufacturer": MANUFACTURER,
        "serialNumber": SERIAL_NUMBER,
        "orderId": "TEST_ORDER",
        "orderUpdateId": 0,
        "nodes": [],
        "edges": []
    }

def generate_2_order():
    """연속 반복 작업 Order 생성"""
    timestamp = datetime.utcnow().strftime("%Y-%m-%dT%H:%M:%SZ")

    # 다양한 픽업-드롭 위치 정의 (x, y, theta)
    pickup_stations = [
        (5.0, 2.0, 0.0),
        (15.0, 5.0, 3.14),
        (8.0, 8.0, 1.57),
        (12.0, 1.0, 0.0)
    ]

    drop_stations = [
        (10.0, 2.0, 0.0),
        (20.0, 8.0, 0.0),
        (3.0, 6.0, -1.57),
        (18.0, 3.0, 3.14)
    ]

    nodes = []
    edges = []
    sequence_id = 1

    # 시작 노드
    nodes.append({
        "nodeId": "START_POSITION",
        "sequenceId": sequence_id,
        "released": True,
        "nodeDescription": "반복 작업 시작점",
        "nodePosition": {
            "x": 0.0,
            "y": 0.0,
            "theta": 0.0,
            "mapId": "warehouse_map",
            "allowedDeviationXY": 0.3
        },
        "actions": []
    })
    sequence_id += 1
    last_node_id = "START_POSITION"
    cycle_count = 4

    for cycle in range(1, cycle_count + 1):
        pickup_idx = (cycle - 1) % len(pickup_stations)
        drop_idx = (cycle - 1) % len(drop_stations)

        pickup_pos = pickup_stations[pickup_idx]
        drop_pos = drop_stations[drop_idx]

        pickup_node_id = f"PICKUP_{cycle}"
        drop_node_id = f"DROP_{cycle}"

        # 시작 → 픽업 엣지
        edges.append({
            "edgeId": f"edge_to_pickup_{cycle}",
            "sequenceId": sequence_id,
            "released": True,
            "startNodeId": last_node_id,
            "endNodeId": pickup_node_id,
            "maxSpeed": 1.2,
            "maxLoadWeight": 0.0,
            "rotationAllowed": True,
            "actions": []
        })
        sequence_id += 1

        # 픽업 노드
        nodes.append({
            "nodeId": pickup_node_id,
            "sequenceId": sequence_id,
            "released": True,
            "nodeDescription": f"사이클 {cycle} 픽업 스테이션",
            "nodePosition": {
                "x": pickup_pos[0],
                "y": pickup_pos[1],
                "theta": pickup_pos[2],
                "mapId": "warehouse_map",
                "allowedDeviationXY": 0.3
            },
            "actions": [
                {
                    "actionId": f"pick_item_{cycle}",
                    "actionType": "pick",
                    "actionDescription": f"사이클 {cycle} 아이템 픽업",
                    "blockingType": "HARD",
                    "actionParameters": [
                        {"key": "item_id", "value": f"ITEM_{cycle}"},
                        {"key": "cycle_num", "value": str(cycle)}
                    ]
                }
            ]
        })
        sequence_id += 1

        # 픽업 → 드롭 엣지
        edges.append({
            "edgeId": f"edge_pickup_to_drop_{cycle}",
            "sequenceId": sequence_id,
            "released": True,
            "startNodeId": pickup_node_id,
            "endNodeId": drop_node_id,
            "maxSpeed": 1.0,
            "maxLoadWeight": 100.0,
            "rotationAllowed": True,
            "actions": []
        })
        sequence_id += 1

        # 드롭 노드
        nodes.append({
            "nodeId": drop_node_id,
            "sequenceId": sequence_id,
            "released": True,
            "nodeDescription": f"사이클 {cycle} 드롭 스테이션",
            "nodePosition": {
                "x": drop_pos[0],
                "y": drop_pos[1],
                "theta": drop_pos[2],
                "mapId": "warehouse_map",
                "allowedDeviationXY": 0.3
            },
            "actions": [
                {
                    "actionId": f"drop_item_{cycle}",
                    "actionType": "drop",
                    "actionDescription": f"사이클 {cycle} 아이템 드롭",
                    "blockingType": "HARD",
                    "actionParameters": [
                        {"key": "item_id", "value": f"ITEM_{cycle}"},
                        {"key": "cycle_num", "value": str(cycle)}
                    ]
                }
            ]
        })
        sequence_id += 1
        last_node_id = drop_node_id

    return {
        "headerId": 1,
        "timestamp": timestamp,
        "version": "2.0.0",
        "manufacturer": MANUFACTURER,
        "serialNumber": SERIAL_NUMBER,
        "orderId": "CONTINUOUS_REPEAT_JOB",
        "orderUpdateId": 0,
        "nodes": nodes,
        "edges": edges
    }

def generate_connection(state="ONLINE"):
    timestamp = datetime.utcnow().strftime("%Y-%m-%dT%H:%M:%SZ")
    return {
        "headerId": 0,
        "timestamp": timestamp,
        "version": "2.0.0",
        "manufacturer": MANUFACTURER,
        "serialNumber": SERIAL_NUMBER,
        "connectionState": state
    }

# -----------------------------
# 메인 루프
# -----------------------------
try:
    while True:
        #client.publish(TOPIC_ORDER, json.dumps(generate_order()), qos=1)
        client.publish(TOPIC_ORDER, json.dumps(generate_2_order()), qos=1)
        #client.publish(TOPIC_CONNECTION, json.dumps(generate_connection()), qos=1)
        time.sleep(5)
except KeyboardInterrupt:
    client.loop_stop()
    client.disconnect()
