import rclpy
from rclpy.node import Node
from inabot_msgs.msg import CanFrame
import struct
import time

class DummyBirdCharger(Node):
    def __init__(self):
        super().__init__('dummy_bird_charger')
        self.pub_can = self.create_publisher(CanFrame, 'can_tx', 10)
        self.node_id = 0x64

    def send_heartbeat(self, state):
        msg = CanFrame()
        msg.can_id = 0x700 + self.node_id
        msg.can_dlc = 1
        msg.data = [state]
        self.pub_can.publish(msg)
        self.get_logger().info(f"Heartbeat: 0x{state:02X}")

    def send_tpdo1(self, current, voltage, max_current, output_status):
        msg = CanFrame()
        msg.can_id = 0x182 + self.node_id
        msg.can_dlc = 8
        msg.data = list(struct.pack('<HHHH',
                                    int(current*256),
                                    int(voltage*256),
                                    int(max_current*16),
                                    output_status))
        self.pub_can.publish(msg)

    def send_tpdo2(self, state, warning, fault, alignment, thermal, power):
        msg = CanFrame()
        msg.can_id = 0x282 + self.node_id
        msg.can_dlc = 8
        msg.data = list(struct.pack('<BBHBBH',
                                    state, warning, fault, alignment, thermal, int(power*16)))
        self.pub_can.publish(msg)

    def run_loop(self):
        # Heartbeat 시퀀스: Boot -> Pre-op -> Operational
        for hb in [0x00, 0x7F, 0x05]:
            self.send_heartbeat(hb)
            time.sleep(1.0)

        # 이후 지속적으로 Operational 상태 유지 + TPDO 데이터 전송
        while rclpy.ok():
            self.send_heartbeat(0x05)
            self.send_tpdo1(current=1.5, voltage=48.0, max_current=5.0, output_status=0x000F)
            self.send_tpdo2(state=2, warning=0, fault=0, alignment=1, thermal=20, power=72.0)
            time.sleep(1.0)

def main(args=None):
    rclpy.init(args=args)
    node = DummyBirdCharger()
    try:
        node.run_loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
