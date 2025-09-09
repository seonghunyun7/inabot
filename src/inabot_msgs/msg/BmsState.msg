# BmsState.msg
# 표준 BatteryState 필드 포함
float32 voltage
float32 current
float32 capacity
float32 charge
float32 percentage
float32 temperature
float32[] cell_voltage

# 추가 정보
uint8 battery_id
float32 hardware_ver
float32 firmware_ver
uint16 cycle_count
uint8 soh
uint8 serial_cells
uint8 battery_status
uint16 protection_flag
uint16 warning_flag
uint16 balancing_status
