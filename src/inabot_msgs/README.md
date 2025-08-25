
├── src/
│   ├── inabot_msgs/         <-- 메시지 패키지
│   └── other_packages/


colcon build --packages-select inabot_msgs
source ~/ros2_ws/install/setup.bash

ysh@ysh-ThinkPad-T16-Gen-4:~/inabot_ws$ source ~/.bashrc 
ysh@ysh-ThinkPad-T16-Gen-4:~/inabot_ws$ ros2 interface show inabot_msgs/msg/ObstacleStatus
# 전방 장애물 상태 메시지
float64 distance   # 전방 장애물까지 거리 (m)
bool stop          # 장애물 정지 플래그
ysh@ysh-ThinkPad-T16-Gen-4:~/inabot_ws$ 
