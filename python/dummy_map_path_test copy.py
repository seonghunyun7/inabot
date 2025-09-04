import can
import struct

# -----------------------------
# 설정
# -----------------------------
can_interface = 'can0'
bus = can.interface.Bus(can_interface, bustype='socketcan')

# 배터리 ID별 메시지 범위
BATTERY_IDS = [1, 2, 3, 4, 5, 6, 7]
MSG_MAP = {
    "pack_info": 0x101,       # current, SOC, SOH, serial cells
    "status": 0x103,          # battery status, protection, warning, balancing
    "cell_temp": 0x104,       # 셀 온도 2개씩
    "cell_volt": 0x105        # 셀 전압 4개씩
}

# -----------------------------
# 메시지 파싱 함수
# -----------------------------
def parse_message(msg_id, data):
    """CAN 데이터 파싱"""
    result = {}

    # 배터리 ID 계산
    battery_id = msg_id >> 8  # 상위 바이트
    base_id = msg_id & 0xFF   # 하위 바이트

    # 0x101~0x108 / 0x701~0x708 규칙
    if base_id in range(1, 9):
        # pack info
        if (msg_id & 0xFF00) in [0x100, 0x700]:
            current = struct.unpack("<h", data[2:4])[0] / 10.0
            soc = struct.unpack("<H", data[4:6])[0] / 10.0
            soh = data[6]
            serial_cells = data[7]
            result.update({
                "battery_id": battery_id,
                "current_A": current,
                "SOC_%": soc,
                "SOH_%": soh,
                "serial_cells": serial_cells
            })

        # status
        if (msg_id & 0xFF) == 0x03:
            battery_status = data[0]
            protection_flag = struct.unpack(">H", data[2:4])[0]
            warning_flag = struct.unpack(">H", data[4:6])[0]
            balancing_status = struct.unpack(">H", data[6:8])[0]
            result.update({
                "battery_status": battery_status,
                "protection_flag": protection_flag,
                "warning_flag": warning_flag,
                "balancing_status": balancing_status
            })

        # cell temperature
        if (msg_id & 0xFF) == 0x04:
            temp1 = struct.unpack("<h", data[0:2])[0] / 10.0
            temp2 = struct.unpack("<h", data[2:4])[0] / 10.0
            result.update({
                "cell_temps": [temp1, temp2]
            })

        # cell voltage
        if (msg_id & 0xFF) in range(0x05, 0x0A):  # 0x105~0x109
            voltages = []
            for i in range(0, 8, 2):
                volt = struct.unpack("<H", data[i:i+2])[0]
                voltages.append(volt)
            result.update({
                "cell_voltages": voltages
            })

    return result if result else None

# -----------------------------
# 수신 루프
# -----------------------------
print("Listening on CAN interface:", can_interface)

try:
    while True:
        msg = bus.recv()
        parsed = parse_message(msg.arbitration_id, msg.data)
        if parsed:
            print(f"ID {hex(msg.arbitration_id)}:", parsed)

except KeyboardInterrupt:
    print("Stopped")
