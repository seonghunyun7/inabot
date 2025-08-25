 # MPC Motion Controller

ROS2 기반의 MPC(Motion Planning Controller)를 위한 패키지입니다.  
`mpc_core`는 별도 라이브러리로 분리되어 있으며, 이 컨트롤러는 OSQP, IPOPT, CppAD 등을 활용하여 동적 최적화 기반 경로 추종 제어를 수행합니다.

---

## 📦 시스템 의존성 설치

아래 의존성은 필수입니다. 빌드 전 반드시 설치해주세요.

### ✅ OSQP (Quadratic Program Solver)

```bash
git clone https://github.com/osqp/osqp.git
cd osqp
mkdir build && cd build
cmake .. && make && sudo make install

    헤더: /usr/local/include/osqp/

    라이브러리: /usr/local/lib/libosqp.so

    설치 후: sudo ldconfig 필수

✅ IPOPT (Interior Point Optimizer)

sudo apt update
sudo apt install coinor-libipopt-dev

    헤더: /usr/include/coin/

    라이브러리: /usr/lib/x86_64-linux-gnu/libipopt.so

⚠️ 오류 발생 시: #include <coin-or/IpIpoptApplication.hpp>
         ->  심볼릭 링크 만들어서 해결하기 (간단함)
         sudo ln -s /usr/include/coin /usr/include/coin-or

         /usr/include/coin-or → /usr/include/coin

⚠️ 오류 발생 시: /usr/include/coin/IpSmartPtr.hpp 수정 필요

// 기존
#ifndef __IPSMARTPTR_HPP__
#define __IPSMARTPTR_HPP__

#ifdef HAVE_CSTDDEF
# include <cstddef>
...
#error "don't have header file for stddef"
#endif

// 수정
#ifndef __IPSMARTPTR_HPP__
#define __IPSMARTPTR_HPP__

#include <cstddef>

✅ Eigen3

sudo apt install libeigen3-dev

    헤더: /usr/include/eigen3/

    라이브러리 없음 (header-only)

✅ CppAD

git clone https://github.com/coin-or/CppAD.git
cd CppAD
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install

    헤더: /usr/local/include/cppad/

    라이브러리: /usr/local/lib/libcppad_lib.so

🛠 빌드 설정
CMakeLists.txt 주요 항목 요약

# CppAD 설정
set(CPPAD_INCLUDE_DIR "/usr/local/include")
set(CPPAD_LIB_DIR "/usr/local/lib")

include_directories(${CPPAD_INCLUDE_DIR})
link_directories(${CPPAD_LIB_DIR})

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-D_GLIBCXX_USE_CXX11_ABI=1)

# MPC_CORE
add_library(mpc_core src/mpc_core.cpp)
target_link_libraries(mpc_core ipopt cppad_lib)

# mpc_core 라이브러리
add_library(mpc_core src/mpc_core.cpp)
target_link_libraries(mpc_core PRIVATE ipopt cppad_lib)

# 실행 노드
add_executable(mpc_motion_controller
  src/mpc_motion_controller_node.cpp
  src/mpc_motion_controller_component.cpp
)
ament_target_dependencies(mpc_motion_controller
  rclcpp geometry_msgs nav_msgs tf2_geometry_msgs tf2_ros visualization_msgs lrbot2_msgs Eigen3
)
target_link_libraries(mpc_motion_controller
  PRIVATE
    mpc_core
    Eigen3::Eigen
)

헤더 및 라이브러리 경로 지정 예

set(CPPAD_INCLUDE_DIR "/usr/local/include")
set(CPPAD_LIB_DIR     "/usr/local/lib")
include_directories(${CPPAD_INCLUDE_DIR})
link_directories(${CPPAD_LIB_DIR})

⚙️ MPC 파라미터 설정 예시 (config/mpc_params.yaml)

mpc_motion_controller:
  ros__parameters:
    mpc_horizon: 10
    mpc_dt: 0.066
    mpc_ref_velocity: 0.3
    max_linear_velocity: 0.6
    max_angular_velocity: 1.0
    goal_threshold: 0.1

    DT: 0.066
    STEPS: 10
    REF_CTE: 0.0
    REF_ETHETA: 0.0
    REF_V: 0.3
    W_CTE: 5000.0
    W_EPSI: 5000.0
    W_V: 1.0
    W_ANGVEL: 100.0
    W_A: 50.0
    W_DANGVEL: 10.0
    W_DA: 10.0
    ANGVEL: 3.0
    MAXTHR: 1.0
    BOUND: 1.0e3

🔨 빌드 및 실행

cd ~/robot_ws
colcon build --packages-select mpc_motion_controller
or
colcon build --symlink-install --packages-select mpc_motion_controller
source install/setup.bash
ros2 run mpc_motion_controller mpc_motion_controller

라이브러리 캐시 갱신:

sudo ldconfig

🧩 환경 변수 추천 (~/.bashrc)

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib

✅ 요약 Checklist

OSQP 설치

IPOPT 설치 (+ 오류 수정)

Eigen3 설치

CppAD 설치

CMake에서 mpc_core 분리 및 링크

파라미터 YAML 작성

    colcon build 및 실행

📁 디렉토리 구성 예시

mpc_motion_controller/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── mpc_params.yaml
├── launch/
│   └── mpc_launch.py
├── src/
│   ├── mpc_core.cpp       # MPC 라이브러리 구현
│   ├── mpc_core.hpp
│   ├── mpc_motion_controller_node.cpp
│   └── mpc_motion_controller_component.cpp
└── README.md


# 

#include "mpc_motion_controller/mpc_core.hpp"
class MpcMotionController : public rclcpp::Node
{
 private:
    MPCCore _mpc;
}

MpcMotionController::MpcMotionController()
{
    // Init parameters for MPC object
    //std::map<string, double> _mpc_params;
    _mpc_params["DT"] = dt_;
    _mpc_params["STEPS"] = mpc_steps_;
    _mpc_params["REF_CTE"] = ref_cte_;
    _mpc_params["REF_ETHETA"] = ref_etheta_;
    _mpc_params["REF_V"] = ref_vel_;
    _mpc_params["W_CTE"] = w_cte_;
    _mpc_params["W_EPSI"] = w_etheta_;
    _mpc_params["W_V"] = w_vel_;
    _mpc_params["W_ANGVEL"] = w_angvel_;
    _mpc_params["W_A"] = w_accel_;
    _mpc_params["W_DANGVEL"] = w_angvel_d_;
    _mpc_params["W_DA"] = w_accel_d_;
    _mpc_params["ANGVEL"] = max_angvel_;
    _mpc_params["MAXTHR"] = max_throttle_;
    _mpc_params["BOUND"] = bound_value_;

    //void LoadParams(const std::map<string, double> &params)
    _mpc.LoadParams(_mpc_params);
}

사용예:
  Eigen::VectorXd state(6);
 
  state << 0, 0, 0, v, cte, etheta; //속도, 횡방향 에러, 종방향 에러 
 
  // Solve MPC Problem
 
  vector<double> mpc_results = _mpc.Solve(state, coeffs);
  // MPC result (all described in car frame), output = (acceleration, w)
  w_ = mpc_results[0];        // radian/sec, angular velocity
  throttle_ = mpc_results[1]; // acceleration
 


# FMS => ROBOT 간 경로 포인트 동작 시나리오

1. 한점에 대한 경로 포인트를 전달한다.
2. 도착지점(10cm) 이면 다음 경로를 보내 준다
3. 이 시나리오에 대해서 현재 MCP 알고리즘과는 맞지 않아 보간 법을 사용.
 - 단, 다음 경로 포인트가 너무 짧으면 문제일 듯.
 
4. 향후 경로 포인트에 대한 동작 시나리오를 FMS와 이야기 해야 된다.
5. 도킹일 경우에는 어떻게??? 짧은 움직임인데..이때는 motion planner를 사용하지 않고
   도킹 전용 motion controller가 필요하지 않을까?

✅ 1. Cubic Spline Interpolation
📌 특징

    부드러운 경로 생성 (연속된 위치, 속도, 가속도).

    각 구간이 3차 다항식으로 연결됨.

    주로 정지점이나 일반적인 경로 생성에 적합.

    구현이 간단한 라이브러리도 많음 (예: tk::spline, Eigen, ROS의 nav_msgs::Path와 연동도 쉬움).

📈 장점

    경로가 자연스럽고 부드럽다.

    일반적인 navigation/motion planning에서는 가장 널리 사용됨.

    적은 제어점으로도 충분히 부드러운 경로 가능.

⚠️ 단점

    방향 정보가 없을 경우: 원하지 않는 곡선이 생길 수 있음.

    경로 끝에서 overshoot 현상이 날 수 있음 (끝점 방향성이 불명확하면).

✅ 2. Hermite Interpolation
📌 특징

    제어점 간의 보간뿐 아니라, **각 점의 방향(속도, 기울기)**도 고려.

    P0, P1 위치뿐 아니라 T0, T1 (각 점의 기울기 벡터)를 사용.

📈 장점

    경로 방향을 정밀하게 제어 가능 (속도, 회전각 정보 있음).

    정밀한 path-following이나, trajectory generation에 적합.

    경사도 정보가 있으면 매우 효과적 (예: MPC에서 사용되는 경로 생성 등).

⚠️ 단점

    각 포인트의 기울기 정보가 필요.

    단순 위치 정보만 있을 때는 부정확하거나 구현이 복잡해짐.

    