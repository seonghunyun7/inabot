#!/usr/bin/env python3
"""
Battery CAN Node (ROS2 Humble, Python)
- Protocol : CAN 2.0A, 250Kbps, 8 bytes
- Message 주기: 1초
- Connector: DSub-9pin Female (Pin2 – CAN L / Pin7 – CAN H)
- Data Format: Big Endian
- CAN 인터페이스가 없을 경우 안전하게 재시도하며 계속 대기
- 수신 데이터는 sensor_msgs/msg/BatteryState 로 publish
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
import can
import struct

class BatteryCANNode(Node):
    def __init__(self):
        super().__init__('battery_can_node')

        self.can_interface = 'can0'
        self.bus = None

        self.get_logger().info(f"Starting Battery CAN node on {self.can_interface}")

        # ROS2 publisher
        self.batt_pub = self.create_publisher(BatteryState, 'battery_state', 10)

        # Timer: 1초마다 read_can 호출
        self.timer = self.create_timer(1.0, self.read_can)

    def connect_can(self):
        """CAN 인터페이스 연결 시도"""
        if self.bus is None:
            try:
                self.bus = can.interface.Bus(self.can_interface, bustype='socketcan')
                self.get_logger().info(f"CAN interface {self.can_interface} connected successfully")
            except OSError:
                self.get_logger().warn(f"CAN interface {self.can_interface} not found. Retrying...")

    def read_can(self):
        """CAN 메시지 수신 및 파싱"""
        if self.bus is None:
            self.connect_can()
            return

        try:
            msg = self.bus.recv(timeout=0.01)  # 10ms 대기
            if msg:
                parsed = self.parse_message(msg.arbitration_id, msg.data)
                if parsed:
                    self.publish_battery_state(parsed)
                    # 로그 출력
                    log_msg = f"[ID {hex(msg.arbitration_id)}] "
                    for k, v in parsed.items():
                        log_msg += f"{k}={v}, "
                    self.get_logger().info(log_msg.rstrip(", "))
        except Exception as e:
            self.get_logger().error(f"CAN read error: {e}")
            self.bus = None

    def parse_message(self, msg_id, data):
        """
        CAN 메시지 파싱 (Big Endian 기준)
        배터리 ID 1 기준:
          0x101 : HW/FW version, Pack capacity, Remain, Cycle count
          0x102 : Pack voltage
          0x103 : Status / Protection / Warning / Balancing
          0x104 : Cell Temperature #1, #2
          0x105~0x109 : Cell Voltages (1~16)
        """
        result = {}
        battery_id = msg_id >> 8  # 상위 8비트 → 배터리 팩 ID
        base_id = msg_id & 0xFF   # 하위 8비트 → 메시지 종류

        # Pack basic info (0x101)
        if base_id == 0x01:
            hw_ver = data[0] * 0.1
            fw_ver = data[1] * 0.1
            pack_capacity = struct.unpack(">H", data[2:4])[0] / 10.0
            pack_remain = struct.unpack(">H", data[4:6])[0] / 10.0
            cycle_count = struct.unpack(">H", data[6:8])[0]

            result.update({
                "battery_id": battery_id,
                "hardware_ver": hw_ver,
                "firmware_ver": fw_ver,
                "pack_capacity_Ah": pack_capacity,
                "pack_remain_Ah": pack_remain,
                "cycle_count": cycle_count
            })

        # Pack voltage (0x102)
        elif base_id == 0x02:
            voltage = struct.unpack(">H", data[0:2])[0] / 10.0
            result.update({
                "battery_id": battery_id,
                "pack_voltage_V": voltage
            })

        # Current / SOC / SOH / Serial cells (0x103~0x108 일부)
        elif base_id in range(1, 9) and (msg_id & 0xFF00) in [0x100, 0x700]:
            current = struct.unpack(">h", data[2:4])[0] / 10.0
            soc = struct.unpack(">H", data[4:6])[0] / 10.0
            soh = data[6]
            serial_cells = data[7]

            result.update({
                "battery_id": battery_id,
                "current_A": current,
                "SOC_%": soc,
                "SOH_%": soh,
                "serial_cells": serial_cells
            })

        # Status (0x103)
        if base_id == 0x03:
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

        # Cell temperature (0x104)
        elif base_id == 0x04:
            temp1 = struct.unpack(">h", data[0:2])[0] / 10.0
            temp2 = struct.unpack(">h", data[2:4])[0] / 10.0
            result.update({"cell_temps": [temp1, temp2]})

        # Cell voltages (0x105~0x109)
        elif base_id in range(0x05, 0x0A):
            voltages = [struct.unpack(">H", data[i:i+2])[0] for i in range(0, 8, 2)]
            result.update({"cell_voltages": voltages})

        return result if result else None

    def publish_battery_state(self, parsed):
        """sensor_msgs/BatteryState 메시지로 변환 후 publish"""
        batt_msg = BatteryState()
        batt_msg.voltage = parsed.get("pack_voltage_V", 0.0)
        batt_msg.current = parsed.get("current_A", 0.0)
        batt_msg.capacity = parsed.get("pack_capacity_Ah", 0.0)
        batt_msg.charge = parsed.get("pack_remain_Ah", 0.0)

        soc = parsed.get("SOC_%", None)
        if soc is not None:
            batt_msg.percentage = soc / 100.0

        temps = parsed.get("cell_temps", None)
        if temps:
            batt_msg.temperature = sum(temps) / len(temps)

        voltages_mV = parsed.get("cell_voltages", None)
        if voltages_mV:
            batt_msg.cell_voltage = [v / 1000.0 for v in voltages_mV]

        self.batt_pub.publish(batt_msg)


def main(args=None):
    rclpy.init(args=args)
    node = BatteryCANNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down battery_can_node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
