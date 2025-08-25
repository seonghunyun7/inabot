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


   ← wheel_separation →
+-----------------------------+
|                             |
|      ○             ○        |   ← 좌우 바퀴 중심 (왼쪽 바퀴, 오른쪽 바퀴)
|      |                  |        |
|      |<-- wheel_base -->|   |   ← 앞뒤 바퀴 축 간 거리
|                             |
+-----------------------------+

         ↑
   wheel_radius (바퀴 반지름)
   ← circumference = 2π × wheel_radius →



물류 이륜 차동 구동 로봇 기준으로 아주 간단한 그림을 그려봤어요.
로봇을 위에서 본 모습이고, 주요 용어 위치 표시해뒀습니다.

       ↑ x (전방)
       |
   -----------------
  |                 |
  |      BODY       |
  |                 |
   -----------------
    O             O
   (L)           (R)
   
   ←--- wheel_separation (좌우 바퀴 중심 간 거리) --->

각 바퀴 반지름: wheel_radius = 바퀴 중심에서 바깥 가장자리까지 거리

    O는 바퀴 (왼쪽 바퀴 L, 오른쪽 바퀴 R)

    wheel_separation는 왼쪽 바퀴 중심에서 오른쪽 바퀴 중심까지의 거리

    wheel_radius는 바퀴의 크기 (반지름)

    로봇 전방 방향은 위쪽 (x축 방향)으로 표시
    
    
wheel_radius: 바퀴 중심부터 바퀴 가장자리까지의 거리
circumference: 바퀴 원주 (2π × wheel_radius)
wheel_separation: 좌우 바퀴 중심 간 거리 (왼쪽 바퀴 중심 ↔ 오른쪽 바퀴 중심)
wheel_base: 앞뒤 바퀴 축 간 거리 (앞 바퀴 축 ↔ 뒤 바퀴 축)