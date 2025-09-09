#BirdChargerStatus.msg
uint8  charging_state
float32 voltage
float32 current
float32 max_current
uint8  output_status
uint8  fault_number
string fault_string
uint8  warning_number
string warning_string

# TPDO2 추가
uint8  alignment
uint8  thermal_usage
float32 charging_power

# Heartbeat 추가
uint8  nmt_state
