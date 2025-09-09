#!/usr/bin/env python3
"""
BMS CAN Dummy Publisher for ROS2 Humble
- Publishes dummy CAN frames using custom BmsCanFrame.msg
- Frame IDs: 0x101 ~ 0x109
- Data Format: Big Endian, 8 bytes
- Purpose: C++ parseMessage() 테스트
    0x102: Pack voltage
    0x103: Current / SOC / SOH / Serial / Protection / Warning / Balance
    0x104: Cell temperatures (2셀)
    0x105~0x109: 각 프레임 4셀 전압
    데이터 형식: Big Endian, 8바이트
"""
import rclpy
from rclpy.node import Node
from inabot_msgs.msg import BmsCanFrame
import struct
import random

class BmsCanDummyPublisher(Node):
    def __init__(self):
        super().__init__('bms_can_dummy_publisher')
        self.pub = self.create_publisher(BmsCanFrame, 'bms_can_frame', 10)
        self.timer = self.create_timer(1.0, self.publish_dummy)

    def publish_dummy(self):
        dummy_frames = []

        # 0x101: Pack basic info
        hw_ver = random.randint(10, 20)            # 1.0~2.0 ×10
        fw_ver = random.randint(10, 30)            # 1.0~3.0 ×10
        capacity = random.randint(400, 600)        # 40~60Ah ×10
        remain = random.randint(200, capacity)     # 20~capacity ×10
        cycles = random.randint(0, 1000)
        data_101 = struct.pack('>BBHHH', hw_ver, fw_ver, capacity, remain, cycles)
        dummy_frames.append((0x101, data_101))

        # 0x102: Pack voltage
        voltage = random.randint(440, 520)         # 44~52V ×10
        data_102 = struct.pack('>H', voltage) + bytes(6)
        dummy_frames.append((0x102, data_102))

        # 0x103: Current / SOC / SOH / Serial / Status / Flags
        bat_status = random.randint(0, 255)        # Byte 0
        current = random.randint(-100, 100)        # -10~10A ×10, Int16
        soc = random.randint(200, 1000)            # 20~100% ×10, UInt16
        soh = random.randint(80, 100)              # Byte 6
        serial = random.randint(1, 16)             # Byte 7
        prot_flag = random.randint(0, 65535)       # Protection Flag, UInt16
        warn_flag = random.randint(0, 65535)       # Warning Flag, UInt16
        balance_status = random.randint(0, 65535)  # Balancing Status, UInt16

        # Byte layout according to BMS spec:
        # 0: Battery status
        # 1: Reserved
        # 2-3: Current
        # 4-5: SOC
        # 6: SOH
        # 7: Serial
        data_103 = struct.pack('>BBhHBB', bat_status, 0, current, soc, soh, serial)
        # Optionally, overwrite last 6 bytes with Protection / Warning / Balance
        data_103 = struct.pack('>B', bat_status) + bytes([0]) \
                   + struct.pack('>HHH', prot_flag, warn_flag, balance_status)
        dummy_frames.append((0x103, data_103))

        # 0x104: Cell temperatures
        t1 = random.randint(200, 400)  # 20~40℃ ×10
        t2 = random.randint(200, 400)
        data_104 = struct.pack('>hh', t1, t2) + bytes(4)
        dummy_frames.append((0x104, data_104))

        # 0x105~0x109: Cell voltages (각 프레임 4셀)
        for frame_id in range(0x105, 0x10A):
            voltages = [random.randint(3200, 4200) for _ in range(4)]
            data = b''.join([struct.pack('>H', v) for v in voltages])
            dummy_frames.append((frame_id, data))

        # Publish frames
        for frame_id, data in dummy_frames:
            msg = BmsCanFrame()
            msg.id = frame_id
            msg.dlc = 8
            msg.data = list(data[:8])
            self.pub.publish(msg)

        self.get_logger().info(f"Published {len(dummy_frames)} dummy CAN frames")

def main(args=None):
    rclpy.init(args=args)
    node = BmsCanDummyPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
