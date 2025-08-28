#!/usr/bin/env python3
"""
Buzzard40 Bird Charger Node (ROS2 Humble, Python)
- Protocol: CANopen
- Node ID: Bird 0x64, Master 0x01
- Baudrate: Default 250Kbps (Supports 125K, 250K, 500K, 1M)
- TPDO / RPDO Mapping:
    RPDO1: 0x180 + NodeID -> Master → System Command
    TPDO1: 0x180 + NodeID -> Bird → System Status 1
    TPDO2: 0x280 + NodeID -> Bird → System Status 2
- Heartbeat: Bird / Master 1000ms
- CANbus Interface: SocketCAN 'can0'
- Charging control and monitoring via SDO
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8, Float32
import can
import canopen
import time

class BirdChargerNode(Node):
    def __init__(self):
        super().__init__('buzzard40_bird_node')
        self.get_logger().info("Initializing Bird Charger Node")

        # -------------------------------
        # CANopen 네트워크 초기화
        # -------------------------------
        self.network = canopen.Network()
        self.BIRD_NODE_ID = 0x64  # 기본 Bird Node ID
        # -------------------------------
        # CAN 버스 연결 및 Bird Node 등록
        # 실패 시 5초 대기 후 재시도
        # -------------------------------
        # -------------------------------
        # NMT: Reset Communication 후 Operational
        # 표준 CANopen NMT 상태:
        # Boot, Pre-operational, Operational, Stopped, Reset, Reset Communication 등
        # -------------------------------
        connected = False
        while not connected:
            try:
                # CAN 인터페이스 연결
                self.network.connect(channel='can0', bustype='socketcan')
                self.get_logger().info("CAN network connected")

                # RemoteNode 등록 (EDS 없이)
                self.bird = canopen.RemoteNode(self.BIRD_NODE_ID, None)
                self.network.add_node(self.bird)

                # NMT 초기화: RESET COMMUNICATION 후 OPERATIONAL
                self.bird.nmt.state = 'RESET COMMUNICATION'
                time.sleep(0.5)
                self.bird.nmt.state = 'OPERATIONAL'
                self.get_logger().info("Bird Node set to OPERATIONAL")

                connected = True

            except (OSError, can.CanError) as e:
                # CAN 장치 없거나 연결 실패 시 경고 후 5초 대기
                self.get_logger().warn(f"CAN 장치 없음 또는 연결 실패: {e}. 5초 후 재시도...")
                time.sleep(5)

        # -------------------------------
        # Heartbeat 콜백
        # 표 7-2: NMT 상태
        # 0x00: Boot, 0x7F: Pre-operational, 0x05: Operational, 0x04: Stopped
        # -------------------------------
        self.network.add_heartbeat_callback(self.heartbeat_cb)

        # -------------------------------
        # TPDO 설정
        # TPDO1: System Status 1 (0x182 + NodeID)
        # TPDO2: System Status 2 (0x282 + NodeID)
        # -------------------------------
        self.setup_tpdo()

        # -------------------------------
        # (충전 상태/전류/전압 등)
        # -------------------------------
        self.pub_charging_state = self.create_publisher(UInt8, 'charging_state', 10)
        self.pub_voltage = self.create_publisher(Float32, 'voltage', 10)
        self.pub_current = self.create_publisher(Float32, 'current', 10)
        self.pub_max_current = self.create_publisher(Float32, 'max_current', 10)
        self.pub_charger_output = self.create_publisher(UInt8, 'output_status', 10)
        self.pub_fault_number = self.create_publisher(UInt8, 'fault_number', 10)
        self.pub_warning_number = self.create_publisher(UInt8, 'warning_number', 10)

        # -------------------------------
        # 주기적 상태 읽기 (1초)
        # -------------------------------
        self.create_timer(1.0, self.read_status)

        # -------------------------------
        # 충전 시작
        # -------------------------------
        # self.start_charging(voltage_v=48.0, current_a=10.0)

    # -------------------------------
    # Heartbeat 콜백
    # -------------------------------
    def heartbeat_cb(self, node_id, state):
        states = {0x00: "Boot", 0x7F: "Pre-operational", 0x05: "Operational", 0x04: "Stopped"}
        self.get_logger().info(f"Heartbeat from node {node_id}: {states.get(state, 'Unknown')}")

    # -------------------------------
    # TPDO 설정 (Transmit PDO from Bird Node)
    # - TPDO1: System Status 1 (0x182 + NodeID)
    #   매핑:
    #     0x2002: Actual Charging Current (Unit = 1/256 A)
    #     0x2101: Actual Charging Voltage (Unit = 1/256 V)
    #     0x4212: Max Available Charger Current (Unit = 1/16 A)
    #     0x2006: Charger Output Status (비트 15~12: 0=Off, 1=On)
    # - TPDO2: System Status 2 (0x282 + NodeID)
    #   매핑:
    #     0x2007: Charger State (0:Waiting, 1:Ready, 2:Charging, 3:Fault)
    #     0x2008: Alignment (0:Bad, 9:Very good)
    #     0x2009: Thermal usage (0:Good, 100:Derating)
    #     0x2051: Fault Number
    #     0x2050: Warning Number
    # - 각 TPDO는 Bird Node가 주기적으로 전송
    # - 콜백 함수(tpdo1_cb, tpdo2_cb)에서 수신 메시지를 처리
    # -------------------------------
    def setup_tpdo(self):
        # TPDO1: System Status 1
        def tpdo1_cb(msg):
            self.get_logger().info(f"TPDO1: {msg.data.hex()}")
        tpdo1 = canopen.pdo.RxPDO(self.bird, cob_id=0x182)
        tpdo1.add_callback(tpdo1_cb)
        self.bird.tpdo.add(tpdo1)

        # TPDO2: System Status 2
        def tpdo2_cb(msg):
            self.get_logger().info(f"TPDO2: {msg.data.hex()}")
        tpdo2 = canopen.pdo.RxPDO(self.bird, cob_id=0x282)
        tpdo2.add_callback(tpdo2_cb)
        self.bird.tpdo.add(tpdo2)

    # -------------------------------
    # 충전 시작 (SDO 기반)
    # 표 7-2: RPDO1 / Charger Command 매핑
    # 0x4200: Connect, 0x4201: Reset Fault, 0x4202: Voltage control
    # 0x2276: Charge Voltage Request, 0x6070: Charge Current Request
    # 0x4203: Run (Start/Stop)
    # -------------------------------
    def start_charging(self, voltage_v=48.0, current_a=10.0):
        voltage_raw = int(voltage_v * 256)  # 매트릭스 단위 변환
        current_raw = int(current_a * 16)
        try:
            self.bird.sdo[0x4200][0].raw = 1  # Connect
            self.bird.sdo[0x4201][0].raw = 1  # Reset Fault
            self.bird.sdo[0x4202][0].raw = 1  # Voltage control
            self.bird.sdo[0x2276][0].raw = voltage_raw
            self.bird.sdo[0x6070][0].raw = current_raw
            self.bird.sdo[0x4203][0].raw = 1  # Start
            self.get_logger().info(f"Charging started: {voltage_v}V, {current_a}A")
        except Exception as e:
            self.get_logger().error(f"Error starting charging: {e}")

    # -------------------------------
    # 충전 정지
    # 0x4203: Run = 0
    # -------------------------------
    def stop_charging(self):
        try:
            self.bird.sdo[0x4203][0].raw = 0
            self.get_logger().info("Charging stopped")
        except Exception as e:
            self.get_logger().error(f"Error stopping charging: {e}")

    # -------------------------------
    # 상태 읽기 (TPDO1 + TPDO2 매핑)
    # -------------------------------
    def read_status(self):
        """
        Bird Node로부터 TPDO1, TPDO2를 통해 전달되는 상태 정보를 읽고
        ROS2 토픽으로 publish.
        
        TPDO2 (0x282 + NodeID) 매핑:
            0x2007: Charger State (0=Waiting, 1=Ready, 2=Charging, 3=Fault)
            0x2008: Alignment (0=Bad, 9=Very Good)
            0x2009: Thermal Usage (0=Good, 100=Derating)
            0x2051: Fault Number
            0x2050: Warning Number

        TPDO1 (0x182 + NodeID) 매핑:
            0x2002: Actual Charging Current [A], 단위=1/256
            0x2101: Actual Charging Voltage [V], 단위=1/256
            0x4212: Max Available Charger Current [A], 단위=1/16
            0x2006: Charger Output Status (비트필드, 0=Off, 1=On)

        퍼블리시되는 ROS2 토픽:
            pub_charging_state: UInt8, 충전 상태
            pub_voltage: Float32, 실제 전압
            pub_current: Float32, 실제 전류
            pub_max_current: Float32, 최대 허용 전류
            pub_charger_output: UInt8, 출력 상태 비트
            pub_fault_number: UInt8, Fault 번호
            pub_warning_number: UInt8, Warning 번호
        """
        try:
            # TPDO2: Charger 상태
            charging_state = self.bird.sdo[0x2007][0].raw      # Charger State
            alignment = self.bird.sdo[0x2008][0].raw          # Alignment
            thermal = self.bird.sdo[0x2009][0].raw            # Thermal Usage
            fault_number = self.bird.sdo[0x2051][0].raw       # Fault Number
            warning_number = self.bird.sdo[0x2050][0].raw     # Warning Number

            # TPDO1: Charger 전류/전압
            actual_current = self.bird.sdo[0x2002][0].raw / 256.0      # 실제 충전 전류 [A]
            actual_voltage = self.bird.sdo[0x2101][0].raw / 256.0      # 실제 충전 전압 [V]
            max_charger_current = self.bird.sdo[0x4212][0].raw / 16.0  # 최대 허용 충전 전류 [A]
            charger_output_status = self.bird.sdo[0x2006][0].raw       # 충전기 출력 상태 비트

            self.pub_charging_state.publish(UInt8(data=charging_state))
            self.pub_voltage.publish(Float32(data=actual_voltage))
            self.pub_current.publish(Float32(data=actual_current))
            self.pub_max_current.publish(Float32(data=max_charger_current))
            self.pub_charger_output.publish(UInt8(data=charger_output_status))
            self.pub_fault_number.publish(UInt8(data=fault_number))
            self.pub_warning_number.publish(UInt8(data=warning_number))

            self.get_logger().info(
                f"State:{charging_state}, Current:{actual_current:.2f}A, Voltage:{actual_voltage:.2f}V, "
                f"MaxCurrent:{max_charger_current:.2f}A, OutputStatus:{charger_output_status}, "
                f"Alignment:{alignment}, Thermal:{thermal}%, Fault:{fault_number}, Warning:{warning_number}"
            )
        except Exception as e:
            self.get_logger().error(f"Error reading status: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = BirdChargerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Bird Node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()