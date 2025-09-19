#!/usr/bin/env python3
"""
Dummy SICK FlexiSoft PLC Modbus TCP server for testing SafetyController Node.
256: Safety Inputs (IN)
257: EMO / Status
258: Safety Outputs (OUT)
"""

from pymodbus.server.sync import StartTcpServer
from pymodbus.device import ModbusDeviceIdentification
from pymodbus.datastore import ModbusSequentialDataBlock, ModbusSlaveContext, ModbusServerContext
import threading
import time
import logging

# ------------------------
# Logging 설정
# ------------------------
logging.basicConfig()
log = logging.getLogger()
log.setLevel(logging.INFO)

# ------------------------
# 더미 레지스터 초기화
# hr 블록을 300개로 늘려 256번지 이상 접근 가능
# ------------------------
store = ModbusSlaveContext(
    di = ModbusSequentialDataBlock(0, [0]*100),   # Discrete Inputs
    co = ModbusSequentialDataBlock(0, [0]*100),   # Coils
    hr = ModbusSequentialDataBlock(0, [0]*300),   # Holding Registers (0~299)
    ir = ModbusSequentialDataBlock(0, [0]*100)    # Input Registers
)
context = ModbusServerContext(slaves=store, single=True)

# ------------------------
# Modbus 서버 정보
# ------------------------
identity = ModbusDeviceIdentification()
identity.VendorName  = 'SICK'
identity.ProductCode = 'FlexiSoft'
identity.ProductName = 'SafetyControllerDummy'
identity.ModelName   = 'DummyPLC'
identity.MajorMinorRevision = '1.0'

# ------------------------
# 더미 데이터 주기적 업데이트
# ------------------------
def update_registers():
    while True:
        # Safety Inputs (256): IN[0]=1, IN[1]=1, 나머지 0
        # EMO / 상태 (257): EMO[0]=1, 나머지 0
        # Safety Outputs (258): OUT[3]=1, 나머지 0
        values = [0b0000000000000011, 0b0000000000000001, 0b0000000000001000]
        store.setValues(3, 256, values)  # 256번지부터 3개 레지스터 업데이트
        log.info("Updated registers 256~258: %s", [bin(v) for v in values])
        time.sleep(0.5)  # 500ms 주기

# 백그라운드 스레드로 레지스터 업데이트
thread = threading.Thread(target=update_registers)
thread.daemon = True
thread.start()

# ------------------------
# Modbus TCP 서버 시작
# ------------------------
log.info("Starting Dummy SICK FlexiSoft PLC Modbus TCP server on 0.0.0.0:5020")
StartTcpServer(context, identity=identity, address=("0.0.0.0", 5020))
