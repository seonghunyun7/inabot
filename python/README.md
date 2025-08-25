          ┌────────────────────┐
          │ DummyOdomAndPathNode│
          └───────┬─────────────┘
                  │ 1. 초기 목표점 생성 (/planned_path 발행)
                  ▼
          ┌────────────────────┐
          │ MpcMotionController │
          └───────┬─────────────┘
                  │ 2. /planned_path 수신 → 경로 보간, MPC 계산 준비
                  │
                  │ 3. /odom 수신
                  │
                  │ 4. 20ms 주기 timerCallback에서 MPC 실행
                  ▼
          ┌────────────────────┐
          │  cmd_vel 발행 (/cmd_vel)  │
          └───────┬─────────────┘
                  │
                  ▼
          ┌────────────────────┐
          │ DummyOdomAndPathNode│
          └───────┬─────────────┘
                  │ 5. /cmd_vel 수신 → 내부 속도 업데이트, 위치 계산
                  │
                  │ 6. 목표점 도달 판단 시 /goal_reached 발행
                  ▼
          ┌────────────────────┐
          │ DummyOdomAndPathNode│ (자기 자신)
          └───────┬─────────────┘
                  │ 7. /goal_reached 수신 → 새로운 목표점 생성 후 /planned_path 발행
                  └───────────────────────────────────┘


🔍 흐름 요약

    DummyOdomAndPathNode → 목표점 1개 발행 (/planned_path)

    MpcMotionController → 경로 기반 MPC 연산, /cmd_vel 발행

    DummyOdomAndPathNode → /cmd_vel 받아서 움직임 반영

    목표점 도달하면 /goal_reached → 새 목표 생성

    반복



구분	목표 거리 (m)	이유
단거리 목표	0.3 ~ 1.0	세밀한 장애물 회피, 안전 주행
중거리 목표	1.0 ~ 3.0	효율적 경로 계획 및 트래젝토리 추종
장거리 목표	5m 이상	글로벌 플래너가 담당, 로컬 플래너와 분리