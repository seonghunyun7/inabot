1. Inverse Kinematics 수식

목표:

geometry_msgs::Twist
(linear_x, angular_z) →
각 바퀴의 조향각(Steering Angle) + 주행 속도(Wheel Velocity)
변수 정의
변수	의미
v	선속도 (linear_x) [m/s]
w	각속도 (angular_z) [rad/s]
L	전/후 바퀴 간 거리 (wheel_base) [m]
r	바퀴 반지름 [m]
θ	조향각 (steering angle) [rad]
v_wheel	바퀴 선속도 [m/s]
🔹 앞바퀴 (전방)

x_front =  L / 2.0;
v_front = v + w * x_front;
θ_front = atan2(w * x_front, v);

🔹 뒷바퀴 (후방)

x_rear = -L / 2.0;
v_rear = v + w * x_rear;
θ_rear = atan2(w * x_rear, v);

✅ 전체 시스템 구조 예시 (간략화)

Twist 메시지
   ↓
inverse kinematics (앞/뒤 휠 조향각 + 속도)
   ↓
+--------------------------+
| 앞바퀴                   |
|  - Steering PID (θ)     |
|  - Traction PID (v)     |
+--------------------------+
| 뒷바퀴                   |
|  - Steering PID (θ)     |
|  - Traction PID (v)     |
+--------------------------+

✅ ROS 메시지 흐름 예시 (간단)

/robot/cmd_vel      [geometry_msgs/Twist]
       ↓
custom_controller_node
       ↓
/front/steering_cmd [std_msgs/Float64]
/front/wheel_cmd    [std_msgs/Float64]
/rear/steering_cmd  [std_msgs/Float64]
/rear/wheel_cmd     [std_msgs/Float64]


=
✅ 1. Inverse Kinematics 수식 정리

아래는 가장 일반적인 구동 방식 3가지의 Inverse Kinematics 수식입니다.
🟦 ① Differential Drive

두 바퀴 기준 선속도(v) 및 각속도(ω)를 각 바퀴 속도로 변환:
vr=v+ω⋅L/2R,vl=v−ω⋅L/2R
vr​=Rv+ω⋅L/2​,vl​=Rv−ω⋅L/2​

    v: 직진 속도

    ω: 회전 속도

    L: 바퀴 간 거리

    R: 바퀴 반지름

🟩 ② Mecanum Drive

x, y, ω를 4개의 바퀴 속도로 변환:
[vflvfrvrlvrr]=1R[1−1−(L+W)11(L+W)11−(L+W)1−1(L+W)]⋅[vxvyω]
​vfl​vfr​vrl​vrr​​
​=R1​
​1111​−111−1​−(L+W)(L+W)−(L+W)(L+W)​
​⋅
​vx​vy​ω​
​

    L, W: 차량의 길이와 폭

    R: 바퀴 반지름

🟥 ③ Omni Drive (3-wheel)

예: 3개의 Omni wheel일 경우, inverse matrix 구성 필요:
[v1v2v3]=J−1[vxvyω]
​v1​v2​v3​​
​=J−1
​vx​vy​ω​
​

(구체적인 J는 바퀴 각도 배치에 따라 달라집니다.)