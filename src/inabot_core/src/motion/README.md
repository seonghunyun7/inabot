자율주행 제어 시스템 구조 다이어그램 (계층 구조 예시)

+------------------------+
|      Global Planner    |   ← 전역 경로 계획 (예: A*, Dijkstra 등)
+------------------------+
            ↓
+------------------------+
|      Local Planner      |   ← 지역 경로 생성 (예: Pure Pursuit, TEB, MPC)
+------------------------+
            ↓
+------------------------+
|      Controller Layer   |   ← 제어 명령 생성 (예: PID, MPC, Linear Controller)
+------------------------+
            ↓
+------------------------+
|    Low-Level Hardware   |   ← 하드웨어 구동 (모터, 센서, CAN 통신 등)
+------------------------+

각 구성 요소 설명:

    Global Planner
    전역적으로 목적지까지 도달하는 최적의 경로를 생성

        예: A*, Dijkstra, RRT

    Local Planner
    현재 위치에서 주변 장애물을 고려한 단기 경로를 생성

        예: Pure Pursuit, MPC, TEB, Dynamic Window Approach (DWA)

    Controller
    로컬 플래너가 만든 경로를 따라가도록 차량을 제어

        예: PID 제어, MPC 제어 등

    Low-Level Hardware
    실제 모터 제어 및 센서 통신

        예: CAN 통신을 통해 제어 명령 전달


Local Planner는 경로 생성자이고,

Controller는 실행자입니다.

단, Pure Pursuit나 MPC는 경로 생성과 제어를 동시에 포함하기 때문에 Local Planner + Controller 역할을 겸하는 하이브리드입니다.