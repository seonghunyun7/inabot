# laser object detect
sudo apt install ros-humble-visualization-msgs
sudo apt install ros-humble-message-filters
sudo apt install ros-humble-vision-msgs
sudo apt install ros-humble-pcl-conversions



LaserScan 입력
       │
       ▼
  이상치 제거
(removeOutliers)
       │
       ▼
LaserScan → PointCloud2 변환
(transformLaserScanToPointCloud)
       │
       ▼
   ROI 필터링
(filterPointCloud)
       │
       ▼
  ROI 안 포인트만 사용
       │
       ▼
  최소 거리 계산
       │
       ▼
  최소 거리 발행
(obstacle_dist_pub_)
       │
       ▼
  최소 거리 반환



           ┌─────────────────────┐
           │   LaserScan 입력    │
           └─────────┬───────────┘
                     │
                     ▼
         ┌─────────────────────┐
         │  이상치 제거         │
         │ removeOutliers()    │
         └─────────┬───────────┘
                     │
                     ▼
         ┌─────────────────────┐
         │ LaserScan → PointCloud2 │
         │ transformLaserScanToPointCloud() │
         └─────────┬───────────┘
                     │
                     ▼
         ┌─────────────────────┐
         │  ROI 필터링          │
         │ filterPointCloud()  │
         │ (로봇 전방 기준)    │
         └─────────┬───────────┘
                     │
                     ▼
         ┌─────────────────────┐
         │  ROI 안 포인트만 사용 │
         │  최소 거리 계산       │
         │  (min_distance)     │
         └─────────┬───────────┘
                     │
                     ▼
         ┌─────────────────────┐
         │ 거리 발행 & 반환      │
         │ obstacle_dist_pub_  │
         │ return min_distance │
         └─────────────────────┘


           Y
           │
           │
           │
           │       장애물
           │       ■
           │
           │
   ┌───────┴─────────┐
   │                   │
   │   로봇 바디 : 로봇 중심        │
   │                   │
   └───────┬─────────┘
           │
           │ robot_front_x_  ← 로봇 앞끝 기준
           ▼
      ROI 시작점
           │
           │
           ▼
      최소 거리 = min_distance : ROI 안 가장 가까운 장애물까지 거리 → 따라서 로봇 전방 끝 기준 거리
           │
           │
       토픽 데이터로 발행


 

 
 1️⃣ 전방 거리 (min_x, max_x)

    min_x = 0.0 m: 로봇 앞부터 감지 시작 → 즉시 장애물 정지 신호 발생

    max_x = 1.0 m: 1 m 앞까지 감지

⚠️ 위험:

    로봇이 빠르게 움직일 경우, 1 m 이내 감지 거리에서는 급정지가 발생할 수 있음.

    예: 0.5 m/s × 1 s → 0.5 m 이동, 감속률에 따라 충돌 가능

💡 권장:

    최소 거리(min_x) 0.2~0.3 m 정도로 띄우고, 감속 곡선을 적용하면 부드럽게 정지 가능

    최대 거리(max_x)는 현재 속도 기준 정지 거리 + 안전 여유 거리 정도로 설정

        예: 0.5 m/s × 1 s 감속 → 약 0.5 m + 안전 마진 0.2 m → 0.7 m 정도

2️⃣ 좌우 범위 (min_y, max_y)

    현재 ±0.6 m → 로봇 폭 0.55 m보다 약간 넓음 → 충분히 커버

    좁은 통로에서는 너무 넓게 잡으면 불필요하게 장애물 감지 발생 → 조금 축소 가능

        예: ±0.35~0.4 m 정도

3️⃣ 속도와 ROI 연동

    감속 곡선을 만들 때, ROI 전방 거리를 그대로 목표 속도 계산에 사용하면 자연스럽게 감속 가능

    예: v_target = k * (distance_to_obstacle - min_safe_distance)



| 속도 $v$ \[m/s] | 감속 거리 $d = v^2 / (2a)$ \[m] | stop\_threshold \[m] | resume\_threshold \[m] |
| ------------- | --------------------------- | -------------------- | ---------------------- |
| 0.2           | 0.04                        | 0.24                 | 0.54                   |
| 0.3           | 0.09                        | 0.29                 | 0.59                   |
| 0.5           | 0.25                        | 0.45                 | 0.75                   |
| 0.8           | 0.64                        | 0.84                 | 1.14                   |
| 1.0           | 1.0                         | 1.2                  | 1.5                    |
| 1.2           | 1.44                        | 1.64                 | 1.94                   |
| 1.5           | 2.25                        | 2.45                 | 2.75                   |


1️⃣ 감속 거리 계산

감속 거리 공식:
d=v22a
d=2av2​

    v=1 m/sv=1m/s (로봇 속도)

    a=0.5 m/s²a=0.5m/s² (최대 감속)

d=122⋅0.5=1 m
d=2⋅0.512​=1m

즉, 로봇이 1 m/s로 이동 중일 때 0.5 m/s²로 감속하면 1 m 거리에서 멈출 수 있습니다.
2️⃣ 안전 마진

센서 오차, MPC 반응 지연, 히스테리시스 고려: +0.2~0.3 m 정도 여유 필요
\text{stop_threshold} \approx 1.2 \text{~} 1.3\,\text{m}

    resume_threshold는 장애물이 안전하게 지나간 후 속도를 재개할 수 있도록 stop_threshold보다 0.3~0.5 m 여유:

\text{resume_threshold} \approx 1.5\,\text{m}
3️⃣ 로컬 플래너/글로벌 플래너 적용

    global_planner_node

obstacle_stop_threshold: 1.2
obstacle_resume_threshold: 1.5

    pure_pursuit / mpc_motion_controller / lidar_obstacle_detector
    ROI 최대 거리(max_x)는 1 m로 두고, stop_threshold = 1.2 m 정도를 참고해서 가감속 적용

좋아요. 그러면 로봇 속도와 LiDAR 감지 범위(1 m)를 기반으로 stop/resume threshold를 정리한 테이블을 만들어 보겠습니다.
가정:

    LiDAR ROI: 0~1 m 전방

    최대 감속: 0.5 m/s² (예시)

    목표: 로컬 플래너가 감속 후 안전하게 정지할 수 있는 STOP 거리

속도 (m/s)	감속거리 d=v2/(2a)d=v2/(2a) (m)	권장 obstacle_stop_threshold_ (m)	권장 obstacle_resume_threshold_ (m)
0.2	0.04	0.1 ~ 0.2	0.15 ~ 0.25
0.5	0.25	0.5 ~ 0.6	0.6 ~ 0.8
1.0	1.0	0.9 ~ 1.0	1.0 ~ 1.2 (ROI 최대치)
1.5	2.25	1.0 (LiDAR 최대)	1.0 ~ 1.2 (히스테리시스 마진)

💡 설명:

    감속거리 = v2/(2a)v2/(2a)


[ LiDAR 센서 ]
       │
       ▼
[ LiDAR 데이터 전처리 ]
 - 노이즈 필터링
 - ROI 지정 (전방 1m, ±0.6m)
       │
       ▼
[ 장애물 거리 계산 ]
 - min_distance 계산
       │
       ▼
[ 글로벌 플래너(GlobalPlanner) ]
 - min_distance 확인
 - stop/resume 판단 (obstacle_stop_)
 - ObstacleStatus 메시지 생성
   { distance, stop }
       │
       ▼
[ 로컬 플래너(Pure Pursuit / MPC) ]
 - ObstacleStatus 구독
 - obstacle_stop_가 true면 감속/정지
   - 거리 기반 감속: decel_ratio 계산
   - decel_ratio == 0 → linear.x=0, angular.z=0
 - 정상 이동 시 기존 경로 추적 (pure pursuit 또는 MPC)
       │
       ▼
[ cmd_vel 발행 ]
 - 최종 속도/회전 명령 퍼블리시
       │
       ▼
[ 로봇 구동 ]
