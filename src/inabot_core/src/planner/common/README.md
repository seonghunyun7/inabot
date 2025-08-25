planner/
 ├── common/
 │    └── path_provider.{hpp,cpp}    ← 경로 저장/조회/콜백 관리
 ├── global/
 │    └── global_planner.cpp         ← PathProvider에 경로 업데이트
 ├── local/
 │    ├── pure_pursuit/
 │    │    └── pure_pursuit.cpp      ← PathProvider에서 경로 읽음
 │    └── mpc_motion_controller/
 │         └── mpc_motion_controller.cpp ← PathProvider에서 경로 읽음


Global Planner: 새 경로를 계산 → path_provider.updatePath(path)

Pure Pursuit: 주행 시 path_provider.getPath()로 경로 가져옴

MPC Motion Controller: 동일하게 path_provider.getPath()로 경로 가져옴

하나의 인스턴스를 std::shared_ptr로 두 컨트롤러에 전달 → 항상 최신 경로 공유


        ┌───────────────────────────────┐
        │   글로벌 플래너 (FMS / Global) │
        │   nav_msgs::msg::Path 생성     │
        └───────────────┬───────────────┘
                        │  updatePath(path)
                        ▼
           ┌──────────────────────────┐
           │   PathProvider (공통)     │
           │  - std::vector<Pose>      │
           │  - thread-safe            │
           └───────┬───────────┬───────┘
                   │           │
     getPose(), getPath()  getPose(), getPath()
                   │           │
                   ▼           ▼
 ┌───────────────────────┐   ┌────────────────────────┐
 │ PurePursuit Controller │   │ MPC Motion Controller  │
 │  - 경로 추종 제어       │   │  - MPC 최적화 제어     │
 └───────────────────────┘   └────────────────────────┘
