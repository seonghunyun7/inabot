
소스 트리



├── inabot_core
│   ├── config
│   ├── include
│   │   ├── imu.hpp
│   │   ├── motor.hpp
│   │   └── odometry.hpp
│   ├── launch
│   │   └── inabot_core.launch.py
│   └── src
│       ├── CMakeLists.txt
│       ├── core.cpp
│       ├── imu
│       │   └── imu.cpp
│       ├── kinematics
│       │   └── kinematics.hpp
│       ├── motor
│       │   └── motor.cpp
│       ├── odom
│       │   └── odometry.cpp
│       ├── package.xml
│       └── README.md
└── ros2_canopen
    ├── canopen
    │   ├── CHANGELOG.rst
    │   ├── CMakeLists.txt
    │   ├── doxygen
    │   │   └── Doxyfile
    │   ├── LICENSE
    │   ├── package.xml
    │   └── sphinx
    │       ├── application
    │       ├── conf.py
    │       ├── developers-guide
    │       ├── images
    │       ├── index.rst
    │       ├── make.bat
    │       ├── Makefile
    │       ├── quickstart
    │       ├── software-tests
    │       ├── _static
    │       └── user-guide
    ├── canopen_402_driver
    │   ├── CHANGELOG.rst
    │   ├── CMakeLists.txt
    │   ├── include
    │   │   └── canopen_402_driver
    │   ├── LICENSE
    │   ├── package.xml
    │   ├── src
    │   │   ├── cia402_driver.cpp
    │   │   ├── command.cpp
    │   │   ├── default_homing_mode.cpp
    │   │   ├── lifecycle_cia402_driver.cpp
    │   │   ├── motor.cpp
    │   │   ├── node_interfaces
    │   │   └── state.cpp
    │   └── test
    │       ├── CMakeLists.txt
    │       ├── master.dcf
    │       └── test_driver_component.cpp
    ├── canopen_base_driver
    │   ├── CHANGELOG.rst
    │   ├── CMakeLists.txt
    │   ├── include
    │   │   └── canopen_base_driver
    │   ├── LICENSE
    │   ├── package.xml
    │   ├── src
    │   │   ├── base_driver.cpp
    │   │   ├── lely_driver_bridge.cpp
    │   │   ├── lifecycle_base_driver.cpp
    │   │   └── node_interfaces
    │   └── test
    │       ├── CMakeLists.txt
    │       ├── master.dcf
    │       ├── test_base_driver_component.cpp
    │       └── test_node_canopen_base_driver_ros.cpp
    ├── canopen_core
    │   ├── CHANGELOG.rst
    │   ├── CMakeLists.txt
    │   ├── ConfigExtras.cmake
    │   ├── include
    │   │   └── canopen_core
    │   ├── launch
    │   │   └── canopen.launch.py
    │   ├── LICENSE
    │   ├── package.xml
    │   ├── readme.md
    │   ├── scripts
    │   │   └── setup_vcan.sh
    │   ├── src
    │   │   ├── configuration_manager.cpp
    │   │   ├── device_container.cpp
    │   │   ├── device_container_error.cpp
    │   │   ├── device_container_node.cpp
    │   │   ├── driver_error.cpp
    │   │   ├── driver_node.cpp
    │   │   ├── lifecycle_manager.cpp
    │   │   ├── master_error.cpp
    │   │   ├── master_node.cpp
    │   │   └── node_interfaces
    │   └── test
    │       ├── bus_configs
    │       ├── CMakeLists.txt
    │       ├── master.dcf
    │       ├── simple.eds
    │       ├── test_canopen_driver.cpp
    │       ├── test_canopen_master.cpp
    │       ├── test_device_container.cpp
    │       ├── test_errors.cpp
    │       ├── test_lifecycle_canopen_driver.cpp
    │       ├── test_lifecycle_canopen_master.cpp
    │       ├── test_lifecycle_manager.cpp
    │       ├── test_node_canopen_driver.cpp
    │       └── test_node_canopen_master.cpp
    ├── canopen_fake_slaves
    │   ├── CHANGELOG.rst
    │   ├── CMakeLists.txt
    │   ├── config
    │   │   ├── cia402_slave.eds
    │   │   └── simple_slave.eds
    │   ├── include
    │   │   └── canopen_fake_slaves
    │   ├── launch
    │   │   ├── basic_slave.launch.py
    │   │   └── cia402_slave.launch.py
    │   ├── LICENSE
    │   ├── package.xml
    │   ├── Readme.md
    │   └── src
    │       ├── basic_slave.cpp
    │       ├── cia402_slave.cpp
    │       └── motion_generator.cpp
    ├── canopen_interfaces
    │   ├── CHANGELOG.rst
    │   ├── CMakeLists.txt
    │   ├── LICENSE
    │   ├── msg
    │   │   └── COData.msg
    │   ├── package.xml
    │   └── srv
    │       ├── COHeartbeatID.srv
    │       ├── CONmtID.srv
    │       ├── CONode.srv
    │       ├── COReadID.srv
    │       ├── CORead.srv
    │       ├── COTargetDouble.srv
    │       ├── COWriteID.srv
    │       └── COWrite.srv
    ├── canopen_master_driver
    │   ├── CHANGELOG.rst
    │   ├── CMakeLists.txt
    │   ├── include
    │   │   └── canopen_master_driver
    │   ├── LICENSE
    │   ├── package.xml
    │   ├── src
    │   │   ├── lely_master_bridge.cpp
    │   │   ├── lifecycle_master_driver.cpp
    │   │   ├── master_driver.cpp
    │   │   └── node_interfaces
    │   └── test
    │       ├── CMakeLists.txt
    │       ├── master.dcf
    │       ├── test_master_driver_component.cpp
    │       ├── test_node_canopen_basic_master.cpp
    │       └── test_node_canopen_basic_master_ros.cpp
    ├── canopen_proxy_driver
    │   ├── CHANGELOG.rst
    │   ├── CMakeLists.txt
    │   ├── config
    │   │   ├── concurrency_test
    │   │   ├── nmt_test
    │   │   ├── pdo_test
    │   │   └── sdo_test
    │   ├── include
    │   │   └── canopen_proxy_driver
    │   ├── LICENSE
    │   ├── package.xml
    │   ├── readme.md
    │   ├── src
    │   │   ├── lifecycle_proxy_driver.cpp
    │   │   ├── node_interfaces
    │   │   └── proxy_driver.cpp
    │   └── test
    │       ├── CMakeLists.txt
    │       ├── master.dcf
    │       ├── test_driver_component.cpp
    │       └── test_node_interface.cpp
    ├── canopen_ros2_control
    │   ├── canopen_ros2_control.xml
    │   ├── CHANGELOG.rst
    │   ├── CMakeLists.txt
    │   ├── include
    │   │   └── canopen_ros2_control
    │   ├── LICENSE
    │   ├── package.xml
    │   ├── src
    │   │   ├── canopen_system.cpp
    │   │   ├── cia402_system.cpp
    │   │   └── robot_system.cpp
    │   └── test
    │       └── test_canopen_system.cpp
    ├── canopen_ros2_controllers
    │   ├── canopen_ros2_controllers.xml
    │   ├── CHANGELOG.rst
    │   ├── CMakeLists.txt
    │   ├── include
    │   │   └── canopen_ros2_controllers
    │   ├── LICENSE
    │   ├── package.xml
    │   ├── src
    │   │   ├── canopen_proxy_controller.cpp
    │   │   └── cia402_device_controller.cpp
    │   └── test
    │       ├── test_canopen_proxy_controller.cpp
    │       ├── test_canopen_proxy_controller.hpp
    │       ├── test_load_canopen_proxy_controller.cpp
    │       ├── test_load_cia402_device_controller.cpp
    │       └── test_load_cia402_robot_controller.cpp
    ├── canopen_tests
    │   ├── CHANGELOG.rst
    │   ├── CMakeLists.txt
    │   ├── config
    │   │   ├── canopen_system
    │   │   ├── cia402
    │   │   ├── cia402_diagnostics
    │   │   ├── cia402_lifecycle
    │   │   ├── cia402_namespaced_system
    │   │   ├── cia402_system
    │   │   ├── robot_control
    │   │   ├── simple
    │   │   ├── simple_diagnostics
    │   │   └── simple_lifecycle
    │   ├── launch
    │   │   ├── analyzers
    │   │   ├── canopen_system.launch.py
    │   │   ├── cia402_diagnostics_setup.launch.py
    │   │   ├── cia402_lifecycle_setup.launch.py
    │   │   ├── cia402_namespaced_system.launch.py
    │   │   ├── cia402_setup.launch.py
    │   │   ├── cia402_system.launch.py
    │   │   ├── proxy_diagnostics_setup.launch.py
    │   │   ├── proxy_lifecycle_setup.launch.py
    │   │   ├── proxy_setup.launch.py
    │   │   ├── proxy_setup_namespaced.launch.py
    │   │   ├── robot_control_setup.launch.py
    │   │   └── view_urdf.launch.py
    │   ├── launch_tests
    │   │   ├── test_cia402_driver.py
    │   │   ├── test_proxy_driver_namespaced.py
    │   │   ├── test_proxy_driver.py
    │   │   ├── test_proxy_lifecycle_driver.py
    │   │   └── test_robot_control.py
    │   ├── LICENSE
    │   ├── package.xml
    │   ├── README.md
    │   ├── ROS_NAMESPACES.md
    │   ├── rviz
    │   │   └── robot_controller.rviz
    │   └── urdf
    │       ├── canopen_system
    │       ├── cia402_system
    │       └── robot_controller
    ├── canopen_utils
    │   ├── canopen_utils
    │   │   ├── cyclic_tester.py
    │   │   ├── __init__.py
    │   │   ├── launch_test_node.py
    │   │   ├── simple_rpdo_tpdo_tester.py
    │   │   └── test_node.py
    │   ├── CHANGELOG.rst
    │   ├── LICENSE
    │   ├── no_tests
    │   │   ├── _test_copyright.py
    │   │   ├── _test_flake8.py
    │   │   └── _test_pep257.py
    │   ├── package.xml
    │   ├── resource
    │   │   └── canopen_utils
    │   ├── setup.cfg
    │   └── setup.py
    ├── Dockerfile
    ├── lely_core_libraries
    │   ├── CHANGELOG.rst
    │   ├── cmake
    │   │   └── lely_core_libraries-extras.cmake
    │   ├── CMakeLists.txt
    │   ├── cogen
    │   │   ├── cogen.py
    │   │   └── __init__.py
    │   ├── LICENSE
    │   ├── package.xml
    │   ├── patches
    │   │   └── 0001-Fix-dcf-tools.patch
    │   └── setup.cfg



블록 다이어그램 (기본 구성)

                ┌────────────────────────────┐
                │       inabot_core          │
                │ ┌────────┬────────┬──────┐ │
                │ │ Motor  │  IMU   │ Odom │ │
                │ │ Node   │  Node  │ Node │ │
                │ └────────┴────────┴──────┘ │
                └────────────┬──────────────┘
                             │
                             ▼
                   ROS2 Topic / Service
                             │
               ┌────────────┴────────────┐
               │      canopen_core       │
               │ (master_node, device_x) │
               └────────────┬────────────┘
                            ▼
                   CANopen Bus (CAN FD)
                            ▼
             ┌────────────┐      ┌────────────┐
             │ Motor 1    │ ...  │ Motor N    │
             │ (EDS 1)    │      │ (EDS 2)    │
             └────────────┘      └────────────┘


🔄 데이터 흐름 요약
흐름 방향	설명
MotorNode → ROS topic /target_velocity	사용자가 설정한 목표 속도
canopen_402_driver (device driver) → CAN bus	목표 속도를 SDO/TPDO로 전달
Motor → CANopen Device → canopen_core	실제 속도, 상태 정보 수신
MotorNode ← /motor_feedback	피드백 받아 odom 계산에 사용
OdometryNode → /odom	로봇 전체 위치 및 방향 산출
ImuNode → /imu	센서 피치/롤/요우 값 송신



🛠️ 각 패키지 역할 정리
패키지/디렉토리	설명
inabot_core/src/motor/	모터 제어 상위 노드 (MotorNode), topic/pub/sub, cmd 처리
inabot_core/src/imu/	IMU 센서 데이터 처리 (ImuNode)
inabot_core/src/odom/	Odometry 계산 및 퍼블리싱 (OdometryNode)
ros2_canopen/canopen_402_driver/	CiA402 profile 기반의 하위 모터 드라이버
ros2_canopen/canopen_core/	CANopen 디바이스 마스터 관리 (master_node, config loader)
config/*.yaml	canopen 구성 파일, 파라미터 로딩
*.eds	각 디바이스마다 정의된 Object Dictionary (CiA 402 등)


🎯 향후 확장 시

    여러 모터 지원: motor_node 안에서 여러 디바이스 노드를 spawn 하거나, launch 파일에서 다중 device ID 구성

    네임스페이스 활용: /left_motor/, /right_motor/ 처럼 motor 별 네임스페이스로 분리

    TF 브로드캐스팅: odom → base_link → imu 프레임 트리 생성


📤 Publish (구독 가능)
토픽 이름	타입	설명
/canopen_master/0x01/motion_state	canopen_interfaces/msg/COState	상태, 모드, 상태머신 정보 등
/canopen_master/0x01/position_actual_value	std_msgs/msg/Int32	현재 위치 (0x6064)
/canopen_master/0x01/velocity_actual_value	std_msgs/msg/Int32	현재 속도 (0x606C)
/canopen_master/0x01/statusword	std_msgs/msg/UInt16	상태워드 (0x6041)
/canopen_master/0x01/error	canopen_interfaces/msg/COError	에러 코드, SDO 에러 등
📥 Subscribe (Publish 해서 명령 보낼 수 있음)
토픽 이름	타입	설명
/canopen_master/0x01/target_position	std_msgs/msg/Int32	목표 위치 설정 (0x607A)
/canopen_master/0x01/target_velocity	std_msgs/msg/Int32	목표 속도 설정 (0x60FF)
/canopen_master/0x01/controlword	std_msgs/msg/UInt16	제어 워드 전송 (0x6040)
/canopen_master/0x01/operation_mode	std_msgs/msg/Int8	모드 설정 (0x6060), 예: 1=PP, 3=PV, 6=HM
✅ 예제: 목표 위치 설정

ros2 topic pub /canopen_master/0x01/target_position std_msgs/msg/Int32 "{data: 15000}"

✅ 예제: 모드 변경

# Profile Position Mode로 전환
ros2 topic pub /canopen_master/0x01/operation_mode std_msgs/msg/Int8 "{data: 1}"

✅ 예제: 현재 속도 모니터링

ros2 topic echo /canopen_master/0x01/velocity_actual_value



int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // config 파일 경로 설정
    std::string config_path = "/path/to/config/fblg2360t_nodes.yaml";  // 실제 경로로 수정

    // motor_node에만 파라미터 오버라이드 예시
    rclcpp::NodeOptions motor_opts;
    motor_opts.parameter_overrides({
        rclcpp::Parameter("config_file", config_path)
    });

    auto motor_node = std::make_shared<MotorNode>(motor_opts);
    auto imu_node   = std::make_shared<ImuNode>(); // 옵션 없이
    auto odom_node  = std::make_shared<OdometryNode>(); // 옵션 없이

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(motor_node);
    executor.add_node(imu_node);
    executor.add_node(odom_node);

    executor.spin();

    rclcpp::shutdown();
    return 0;
}

✅ MotorNode 클래스 쪽도 이렇게 수정해야 함

// motor.hpp
class MotorNode : public rclcpp::Node
{
public:
    explicit MotorNode(const rclcpp::NodeOptions & options)
    : Node("motor_node", options)
    {
        std::string config_file;
        this->declare_parameter<std::string>("config_file", "");
        this->get_parameter("config_file", config_file);

        RCLCPP_INFO(this->get_logger(), "Loaded config: %s", config_file.c_str());
        // config_file 경로 기반으로 파라미터 로딩 작업 수행
    }
};



예시: 슬레이브 노드 0x01, 0x02 위치 실제값 구독 (Int32 메시지)

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

class EncoderSubscriber : public rclcpp::Node
{
public:
  EncoderSubscriber() : Node("encoder_subscriber")
  {
    left_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "/canopen_master/0x01/position_actual_value", 10,
      std::bind(&EncoderSubscriber::leftEncoderCallback, this, std::placeholders::_1));

    right_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "/canopen_master/0x02/position_actual_value", 10,
      std::bind(&EncoderSubscriber::rightEncoderCallback, this, std::placeholders::_1));
  }

private:
  void leftEncoderCallback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Left encoder ticks: %d", msg->data);
    left_ticks_ = msg->data;
  }

  void rightEncoderCallback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Right encoder ticks: %d", msg->data);
    right_ticks_ = msg->data;
  }

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr left_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr right_sub_;

  int32_t left_ticks_ = 0;
  int32_t right_ticks_ = 0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EncoderSubscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

https://zhuanlan.zhihu.com/p/695674236

ros2_canopen과 MotorNode 통신 구조 핵심
1) ros2_canopen에서 제공하는 토픽들 (슬레이브 상태 & 제어)

    제어용 Subscribe 토픽

        /canopen_master/0x01/target_position (Int32)

        /canopen_master/0x01/target_velocity (Int32)

        /canopen_master/0x01/controlword (UInt16)

        /canopen_master/0x01/operation_mode (Int8)
        → MotorNode는 이 토픽들에 메시지를 publish하여 모터를 제어합니다.

    상태용 Publish 토픽

        /canopen_master/0x01/position_actual_value (Int32) → 왼쪽 모터 위치(엔코더)

        /canopen_master/0x01/velocity_actual_value (Int32)

        /canopen_master/0x01/statusword (UInt16)

        /canopen_master/0x01/error (COError)
        → MotorNode는 이 토픽들을 subscribe 해서 상태를 받아 처리할 수 있습니다.

    ※ 슬레이브 노드 ID가 0x01, 0x02 등 다르면 각각 다르게 구독/발행할 토픽 이름이 달라집니다.

2) MotorNode에서 할 일 (전체 흐름)
작업	ros2_canopen 쪽 토픽 / 방법
cmd_vel (로봇 속도 명령) 받아서	MotorNode에서 기구학 계산 (휠 속도/위치 목표 생성)
각 휠별 목표 위치/속도 계산 후	/canopen_master/0x01/target_position (또는 velocity) 퍼블리시
ros2_canopen에서 슬레이브 상태 읽기	/canopen_master/0x01/position_actual_value 구독
엔코더 위치 변화량 계산 후 odometry 생성	내부적으로 계산 (MotorNode 내부 구현)
odom 토픽 발행	nav_msgs/msg/Odometry 등
3) 구체적인 토픽 이름 예시
휠	슬레이브 ID	목표 제어 토픽	상태 읽기 토픽
왼쪽 휠	0x01	/canopen_master/0x01/target_position	/canopen_master/0x01/position_actual_value
오른쪽 휠	0x02	/canopen_master/0x02/target_position	/canopen_master/0x02/position_actual_value
4) 서비스 호출?

    기본적으로 모터 제어는 토픽 기반(Publish/Subscribe) 으로 동작합니다.

    서비스 호출 형태는 보통 슬레이브 설정 변경 등 특정 명령에 사용되며, 주행 제어에는 사용하지 않습니다.

5) 요약: MotorNode 동작 예

[cmd_vel] --(기구학 계산)--> 목표 휠 위치/속도 메시지 publish -->
  /canopen_master/0x01/target_position (왼쪽 휠)
  /canopen_master/0x02/target_position (오른쪽 휠)

[ros2_canopen 슬레이브] --(실제 CAN 통신)--> 슬레이브 모터 제어

[ros2_canopen 마스터] --(CAN 수신)--> 
  /canopen_master/0x01/position_actual_value (왼쪽 휠 엔코더 위치) publish
  /canopen_master/0x02/position_actual_value (오른쪽 휠 엔코더 위치) publish

[MotorNode] subscribe 위 2개 토픽 → 엔코더 값 받아 odom 계산 및 상태 확인


    fblg2360t_nodes.yaml 예시

canopen:
  slaves:
    - id: 1
      name: fblg2360t_ch1
      eds_file: "config/fblg2360t_ch1.eds"
    - id: 2
      name: fblg2360t_ch2
      eds_file: "config/fblg2360t_ch2.eds"
 
 

return LaunchDescription([
    Node(
        package='canopen_master_driver',
        executable='canopen_master_node',
        name='canopen_master',
        output='screen',
        parameters=['config/fblg2360t_nodes.yaml'],
    ),
    Node(
        package='inabot_core',
        executable='inabot_core',
        #name='motor_driver_node',
        output='screen',
        parameters=['config/motor_params.yaml'],
    )
])


1. Odometry 연동 (로봇 위치 추정)

actual_velocity 또는 actual_position → encoder_sub → Odometry 메시지 퍼블리시 노드를 별도 생성

/canopen_master/0x01/position_actual_value → left_wheel_encoder
/canopen_master/0x02/position_actual_value → right_wheel_encoder



✅ 1. 기본 구조 요약
엔코더 피드백 흐름 (TPDO 통해 수신):

[CANopen Device (모터)] 
   → [TPDO: Position_Actual_Value, Velocity_Actual_Value 등 전송]
   → [canopen_402_driver 노드가 이 데이터 수신 및 ROS topic 퍼블리시]
   → [/motor_xx/position_actual_value 토픽으로 노출]

✅ 2. 읽을 수 있는 ROS 2 토픽

ros2_canopen의 canopen_402_driver가 자동으로 아래 토픽들을 퍼블리시합니다.
ROS 토픽 이름	설명
/motor_left/position_actual_value	모터 위치 (엔코더 값)
/motor_left/velocity_actual_value	모터 속도
/motor_left/statusword	상태워드
/motor_left/state	장치 상태(ex. OPERATION ENABLE)

(모터 이름은 YAML 구성에서 정함 – 예: motor_left, motor1 등)
✅ 3. 예시 코드: 엔코더 값 구독

Odometry 계산에 사용할 엔코더 값을 MotorNode 또는 OdometryNode에서 구독:

#include "std_msgs/msg/int32.hpp"

rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr left_encoder_sub_;
rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr right_encoder_sub_;

void encoder_callback_left(const std_msgs::msg::Int32::SharedPtr msg)
{
    RCLCPP_INFO(get_logger(), "Left Encoder: %d", msg->data);
    // 여기서 odom 계산에 사용
}

void encoder_callback_right(const std_msgs::msg::Int32::SharedPtr msg)
{
    RCLCPP_INFO(get_logger(), "Right Encoder: %d", msg->data);
    // 여기서 odom 계산에 사용
}

left_encoder_sub_ = this->create_subscription<std_msgs::msg::Int32>(
    "/motor_left/position_actual_value", 10,
    std::bind(&YourClass::encoder_callback_left, this, std::placeholders::_1)
);

right_encoder_sub_ = this->create_subscription<std_msgs::msg::Int32>(
    "/motor_right/position_actual_value", 10,
    std::bind(&YourClass::encoder_callback_right, this, std::placeholders::_1)
);

✅ 4. YAML 구성 확인 (eds 기반 매핑)

canopen_402_driver 가 사용할 수 있도록 EDS 파일과 YAML 설정이 필요합니다.

motor_left:
  dcf: "config/motor_left.eds"
  driver: "canopen_402_driver"
  bus: "can0"
  node_id: 1
  mappings:
    - position_actual_value
    - velocity_actual_value

    이 설정을 통해 ros2_canopen은 CAN TPDO에서 값을 읽어 /motor_left/position_actual_value 같은 ROS 토픽으로 변환해줍니다.

✅ 5. 실제 구성 흐름 요약

CANopen 모터
 └─> Position_Actual_Value (TPDO)
      └─> canopen_402_driver 수신
           └─> ROS 토픽으로 노출 (/motor_xx/position_actual_value)
                └─> MotorNode 또는 OdometryNode에서 구독

🧩 요약

    엔코더 데이터는 canopen_402_driver가 자동으로 /position_actual_value 토픽으로 퍼블리시합니다.

    사용자는 해당 토픽을 구독하여 Odometry 계산에 사용할 수 있습니다.


    1. fblg2360t_nodes.yaml 예시 (2개 슬레이브 등록)

canopen:
  slaves:
    - id: 1
      name: fblg2360t_ch1
      eds_file: "config/fblg2360t_ch1.eds"
      # 추가 설정들...
    - id: 2
      name: fblg2360t_ch2
      eds_file: "config/fblg2360t_ch2.eds"
      # 추가 설정들...

    id : CANopen 네트워크상의 노드 ID (1, 2)

    eds_file : 각각의 EDS 파일 경로

    노드 이름과 기타 파라미터도 각각 설정 가능

    =========================


    fbl2360.eds (예: Roboteq, Maxon 등)

[Device]
DeviceName=Roboteq FBL2360T
DeviceType=0
VendorName=Roboteq Inc
VendorNumber=12345
ProductNumber=0x0010
RevisionNumber=0x0001

[1000]
ParameterName=Device Type
ObjectType=0x07
DataType=0x0007
AccessType=ro
DefaultValue=0x00000000

[6041]
ParameterName=Statusword
ObjectType=0x07
DataType=0x0006
AccessType=ro

[6040]
ParameterName=Controlword
ObjectType=0x07
DataType=0x0006
AccessType=rw

[6060]
ParameterName=Modes of Operation
ObjectType=0x07
DataType=0x0005
AccessType=rw

[6061]
ParameterName=Modes of Operation Display
ObjectType=0x07
DataType=0x0005
AccessType=ro

[6064]
ParameterName=Position Actual Value
ObjectType=0x07
DataType=0x0005
AccessType=ro

[606C]
ParameterName=Velocity Actual Value
ObjectType=0x07
DataType=0x0005
AccessType=ro

[60FF]
ParameterName=Target Velocity
ObjectType=0x07
DataType=0x0005
AccessType=rw


2. YAML 설정 파일 (Node config)

이 파일은 ros2_canopen에서 각 노드를 구성할 때 사용됩니다.
📁 fbl2360_nodes.yaml

master:
  node_id: 1
  driver: canopen_master_driver
  package: canopen_master_driver

motor_left:
  node_id: 2
  driver: canopen_402_driver
  package: canopen_402_driver
  dcf: "config/fbl2360.eds"
  bus: "can0"
  mappings:
    - position_actual_value
    - velocity_actual_value
    - statusword
    - modes_of_operation_display

motor_right:
  node_id: 3
  driver: canopen_402_driver
  package: canopen_402_driver
  dcf: "config/fbl2360.eds"
  bus: "can0"
  mappings:
    - position_actual_value
    - velocity_actual_value
    - statusword
    - modes_of_operation_display

📌 주요 필드 설명:
키	설명
node_id	CANopen 상에서의 Node ID (모터마다 다르게)
driver	사용할 ROS2 CANopen 드라이버
package	드라이버가 속한 패키지
dcf	사용할 EDS 파일 경로
bus	연결된 CAN 인터페이스 (예: can0)
mappings	읽거나 쓰고 싶은 Object Dictionary key 이름들
✅ 3. 구성 구조

your_package/
├── config/
│   ├── fbl2360.eds
│   └── fbl2360_nodes.yaml
├── launch/
│   └── bringup_canopen.launch.py

✅ 4. Launch 예시 (bringup용)

# bringup_canopen.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="canopen_ros2_control",  # or your launchable wrapper
            executable="ros2_canopen_node",
            name="canopen_bridge",
            parameters=["config/fbl2360_nodes.yaml"]
        )
    ])

✅ 5. 예상 노출 토픽
ROS 2 Topic	설명
/motor_left/position_actual_value	왼쪽 모터 위치 (엔코더 값)
/motor_left/velocity_actual_value	왼쪽 모터 속도
/motor_left/state	장치 상태
/motor_right/position_actual_value	오른쪽 모터 위치


==

    목표:

    cmd_vel을 받아 각 휠(좌/우)의 회전 속도로 변환

    해당 속도를 모터에 전달

    모터에서 제공하는 엔코더 값을 읽어 로봇의 주행 정보 획득

즉, 모터 제어 + 엔코더 읽기까지만 ros2_canopen으로 구성하고, **기구학 변환(Twist ↔︎ 휠 속도)**는 직접 구성하는 아키텍처입니다.
✅ 구성 요약

[cmd_vel (Twist 메시지)]      [odom (Wheel Encoder 기반)]
         ↓                                ↑
[기구학 노드: diff_drive_kinematics or custom]
         ↓                                ↑
[ros2_canopen]  <—>  [Roboteq 모터 컨트롤러]

✅ 구성 요소 설명
1. ros2_canopen 설정

ros2_canopen은 다음 노드로 구성됩니다:

    canopen_master_node

    canopen_proxy_driver_node (모터 수만큼)

src/
 └── your_robot/
      ├── config/
      │     └── roboteq_nodes.yaml        ← 전체 node 구성
      │     └── left_motor.eds            ← roboteq EDS 파일
      │     └── right_motor.eds
      └── launch/
            └── roboteq_canopen.launch.py

예시: roboteq_nodes.yaml

can_interface_name: "can0"
master:
  node_id: 1
  dcf: false

nodes:
  left_motor:
    node_id: 2
    dcf: false
    eds: "config/left_motor.eds"

  right_motor:
    node_id: 3
    dcf: false
    eds: "config/right_motor.eds"

2. 기구학 노드 구성 (Twist ↔ Wheel)

cmd_vel 메시지를 받아서 각 휠의 속도로 변환하는 노드를 별도로 만들어야 합니다.
예시 동작:

    cmd_vel → 좌/우 바퀴 angular velocity 계산 → ros2_canopen로 속도 전송

    엔코더 → 각 바퀴 회전량 계산 → odom 계산 후 /odom 퍼블리시

예시 노드 이름:

    diff_drive_controller 또는 custom_kinematics_node

cmd_vel  →  휠 속도 계산  →  roboteq via CANopen
encoder  →  휠 회전량     →  odometry 변환

3. 실시간 데이터 흐름

                cmd_vel
                   ↓
     ┌────────────────────────┐
     │  diff_drive_kinematics │
     └────────────────────────┘
       ↓                     ↓
  left_wheel            right_wheel
     ↓                       ↓
[left_motor PDO]     [right_motor PDO]
     ↓                       ↓
 [Roboteq Controller via CANOpen]
     ↑                       ↑
  encoder pos           encoder pos
       \_______________________/
                   ↑
                odometry

✅ 추천 작업 순서

    roboteq EDS 파일 준비 (Roboteq에서 제공)

    ros2_canopen으로 각 모터 상태 읽기/쓰기 확인 (별도 노드 없이 ros2 topic echo로 확인 가능)

    cmd_vel → 휠 속도 변환 노드 작성

    엔코더 값 → odom 계산 노드 작성

    필요한 경우 TF /odom → /base_link 브로드캐스트

✅ 참고 패키지

    ros2_canopen (https://github.com/ros-industrial/ros2_canopen)

    diff_drive_controller (controller는 필요 없다면 생략 가능)

    robot_localization (추후 IMU 등과 함께 사용할 경우)

✅ 예제 소스 참고

GitHub에 있는 간단한 예제:

    ros2_canopen_examples

    canopen_proxy_driver

구성 예제나 코드 스니펫이 필요하면 말씀해 주세요.
특히 cmd_vel → wheel rpm 변환하는 코드와 odom 생성 코드도 드릴 수 있습니다.
You said:
c402인지 그거 프로토콜이 지원하는 모터야
ChatGPT said:

좋습니다! Roboteq 모터 컨트롤러가 CANopen C402 프로파일 (CiA 402) 을 지원한다면, ros2_canopen 패키지만으로 모터 제어와 엔코더 값 획득이 가능합니다.

즉, 로봇의 저수준 제어 노드 없이도 cmd_vel → 각 바퀴의 속도 제어, 그리고 엔코더 값 읽기가 가능합니다. 다만 기구학 계산(cmd_vel을 각 휠 속도로 변환하거나 odom으로 역변환)은 여전히 별도 노드에서 처리해줘야 합니다.
✅ 구성 요약 (CiA402 기반)

┌───────────────┐     ┌────────────────┐
│ cmd_vel       │     │ encoder pos    │
└────┬──────────┘     └─────┬──────────┘
     ↓                        ↑
┌────────────┐        ┌────────────┐
│ 기구학 노드│◀──────▶│  ros2_canopen │
└────────────┘        └────────────┘
     ↓                        ↑
[wheel velocity]       [wheel position]
     ↓                        ↑
[  Roboteq via CiA402 over CANopen  ]

✅ 핵심 구성 요소
1. ros2_canopen 노드 설정

CiA402 프로파일을 지원하는 generic driver가 포함되어 있습니다.
roboteq_nodes.yaml

can_interface_name: "can0"
master:
  node_id: 1

nodes:
  left_motor:
    node_id: 2
    driver: "ros2_canopen::Cia402Device"
    eds: "config/left_motor.eds"

  right_motor:
    node_id: 3
    driver: "ros2_canopen::Cia402Device"
    eds: "config/right_motor.eds"

2. Cia402Device가 퍼블리시하는 인터페이스

기본적으로 다음 토픽 및 서비스 사용 가능:

    /left_motor/motor_position (std_msgs/Int32 또는 custom)

    /left_motor/motor_velocity_command (std_msgs/Float64)

    /right_motor/... 동일

✅ 기구학 노드 구성 예시

직접 만드실 수도 있고, 간단한 C++/Python 노드에서 다음 기능을 구현하면 됩니다:

    /cmd_vel 구독

    차체 파라미터 (wheel_base, wheel_radius) 기반으로 좌/우 휠 속도 계산

    /left_motor/motor_velocity_command, /right_motor/... 퍼블리시

    /left_motor/motor_position 구독 → 엔코더 값 적분

    /odom 메시지 퍼블리시

✅ Twist → Wheel Velocity 변환 예시 (Python)

def twist_to_wheel_velocity(cmd_vel, wheel_base, wheel_radius):
    v = cmd_vel.linear.x
    w = cmd_vel.angular.z
    v_left  = (v - (wheel_base / 2.0) * w) / wheel_radius
    v_right = (v + (wheel_base / 2.0) * w) / wheel_radius
    return v_left, v_right

✅ 보너스: 오픈소스 예제

ros2_canopen 공식 repo의 Cia402 예제도 확인해보세요:

📁 https://github.com/ros-industrial/ros2_canopen/tree/humble/canopen_proxy_driver
✅ 마무리 구성 요약

    ros2_canopen + Cia402Device = 모터 제어 & 엔코더 데이터 OK

    cmd_vel/odom 변환 로직은 자체 기구학 노드에서 구현

    별도의 controller_manager나 diff_drive_controller 없이도 동작 가능

원하시면 다음 예제 드릴 수 있습니다:

    cmd_vel → 휠 속도 퍼블리시하는 ROS 2 노드 (Python/C++)

    휠 encoder → odom 계산하는 노드

    launch 파일 구성 예제

필요한 내용 말씀 주세요!


✅ 구성 요약 (CiA402 기반)

┌───────────────┐     ┌────────────────┐
│ cmd_vel       │     │ encoder pos    │
└────┬──────────┘     └─────┬──────────┘
     ↓                        ↑
┌────────────┐        ┌────────────┐
│ 기구학 노드│◀──────▶│  ros2_canopen │
└────────────┘        └────────────┘
     ↓                        ↑
[wheel velocity]       [wheel position]
     ↓                        ↑
[  Roboteq via CiA402 over CANopen  ]

✅ 핵심 구성 요소
1. ros2_canopen 노드 설정

CiA402 프로파일을 지원하는 generic driver가 포함되어 있습니다.
roboteq_nodes.yaml

can_interface_name: "can0"
master:
  node_id: 1

nodes:
  left_motor:
    node_id: 2
    driver: "ros2_canopen::Cia402Device"
    eds: "config/left_motor.eds"

  right_motor:
    node_id: 3
    driver: "ros2_canopen::Cia402Device"
    eds: "config/right_motor.eds"

2. Cia402Device가 퍼블리시하는 인터페이스

기본적으로 다음 토픽 및 서비스 사용 가능:

    /left_motor/motor_position (std_msgs/Int32 또는 custom)

    /left_motor/motor_velocity_command (std_msgs/Float64)

    /right_motor/... 동일

✅ 기구학 노드 구성 예시

직접 만드실 수도 있고, 간단한 C++/Python 노드에서 다음 기능을 구현하면 됩니다:

    /cmd_vel 구독

    차체 파라미터 (wheel_base, wheel_radius) 기반으로 좌/우 휠 속도 계산

    /left_motor/motor_velocity_command, /right_motor/... 퍼블리시

    /left_motor/motor_position 구독 → 엔코더 값 적분

    /odom 메시지 퍼블리시

✅ Twist → Wheel Velocity 변환 예시 (Python)

def twist_to_wheel_velocity(cmd_vel, wheel_base, wheel_radius):
    v = cmd_vel.linear.x
    w = cmd_vel.angular.z
    v_left  = (v - (wheel_base / 2.0) * w) / wheel_radius
    v_right = (v + (wheel_base / 2.0) * w) / wheel_radius
    return v_left, v_right

✅ 보너스: 오픈소스 예제

ros2_canopen 공식 repo의 Cia402 예제도 확인해보세요:

📁 https://github.com/ros-industrial/ros2_canopen/tree/humble/canopen_proxy_driver
✅ 마무리 구성 요약

    ros2_canopen + Cia402Device = 모터 제어 & 엔코더 데이터 OK

    cmd_vel/odom 변환 로직은 자체 기구학 노드에서 구현

    별도의 controller_manager나 diff_drive_controller 없이도 동작 가능

원하시면 다음 예제 드릴 수 있습니다:

    cmd_vel → 휠 속도 퍼블리시하는 ROS 2 노드 (Python/C++)

    휠 encoder → odom 계산하는 노드

    launch 파일 구성 예제

필요한 내용 말씀 주세요!

✅ 1. diff_drive_kinematics_node.cpp

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64.hpp"

class DiffDriveKinematicsNode : public rclcpp::Node
{
public:
    DiffDriveKinematicsNode()
        : Node("diff_drive_kinematics_node")
    {
        // Declare and get parameters
        this->declare_parameter("wheel_radius", 0.05);  // meters
        this->declare_parameter("wheel_base", 0.3);     // meters

        wheel_radius_ = this->get_parameter("wheel_radius").as_double();
        wheel_base_ = this->get_parameter("wheel_base").as_double();

        // Publishers
        left_wheel_pub_ = this->create_publisher<std_msgs::msg::Float64>("left_motor/motor_velocity_command", 10);
        right_wheel_pub_ = this->create_publisher<std_msgs::msg::Float64>("right_motor/motor_velocity_command", 10);

        // Subscriber
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            std::bind(&DiffDriveKinematicsNode::cmdVelCallback, this, std::placeholders::_1));
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        double v = msg->linear.x;
        double w = msg->angular.z;

        // Differential drive kinematics
        double v_left = (v - (wheel_base_ / 2.0) * w) / wheel_radius_;
        double v_right = (v + (wheel_base_ / 2.0) * w) / wheel_radius_;

        std_msgs::msg::Float64 left_msg;
        std_msgs::msg::Float64 right_msg;
        left_msg.data = v_left;
        right_msg.data = v_right;

        left_wheel_pub_->publish(left_msg);
        right_wheel_pub_->publish(right_msg);
    }

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_wheel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_wheel_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    double wheel_radius_;
    double wheel_base_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DiffDriveKinematicsNode>());
    rclcpp::shutdown();
    return 0;
}

✅ 2. CMakeLists.txt 예시

cmake_minimum_required(VERSION 3.5)
project(diff_drive_kinematics)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(diff_drive_kinematics_node src/diff_drive_kinematics_node.cpp)

ament_target_dependencies(diff_drive_kinematics_node
  rclcpp
  std_msgs
  geometry_msgs
)

install(TARGETS
  diff_drive_kinematics_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()

✅ 3. package.xml 예시

<package format="3">
  <name>diff_drive_kinematics</name>
  <version>0.1.0</version>
  <description>Diff drive kinematics node</description>
  <maintainer email="your@email.com">Your Name</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>

</package>

✅ 4. launch 예제 (diff_drive.launch.py)

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='diff_drive_kinematics',
            executable='diff_drive_kinematics_node',
            name='diff_drive_kinematics_node',
            parameters=[{
                'wheel_radius': 0.05,
                'wheel_base': 0.3
            }],
            output='screen',
        )
    ])

✅ 퍼블리시 토픽 확인

    /cmd_vel → 구독

    /left_motor/motor_velocity_command → 퍼블리시

    /right_motor/motor_velocity_command → 퍼블리시

✅ 다음 단계

이제 roboteq CANopen 노드가 std_msgs::msg::Float64로 속도 명령을 받을 수 있도록 설정되어 있으면 연동됩니다.
원하시면 반대로 엔코더 값을 읽어 odom을 퍼블리시하는 노드도 만들어 드릴게요.


✅ 폴더 구조 예시

your_ros2_ws/
└── src/
    └── your_robot_package/
        ├── config/
        │   ├── roboteq_nodes.yaml
        │   ├── left_motor.eds
        │   └── right_motor.eds
        └── launch/
            └── roboteq_canopen.launch.py

✅ 1. roboteq_nodes.yaml (구성 파일)

can_interface_name: "can0"  # 실제 CAN 장치명 확인 필요 (ex. slcan0 등)
master:
  node_id: 1

nodes:
  left_motor:
    node_id: 2
    driver: "ros2_canopen::Cia402Device"
    eds: "config/left_motor.eds"

  right_motor:
    node_id: 3
    driver: "ros2_canopen::Cia402Device"
    eds: "config/right_motor.eds"

    node_id는 각 Roboteq 모터 컨트롤러에 설정된 CAN Node ID입니다.
    EDS는 Roboteq에서 제공한 .eds 파일을 사용하세요.

✅ 2. roboteq_canopen.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='canopen_master_driver',
            executable='canopen_master_node',
            name='canopen_master',
            output='screen',
            parameters=['config/roboteq_nodes.yaml']
        )
    ])

    parameters 경로는 launch 실행 시의 기준 경로가 workspace root이므로 상대경로로 지정하거나 절대경로로 변경 가능.

✅ 3. 실행 명령

ros2 launch your_robot_package roboteq_canopen.launch.py

✅ 실행 확인 시

    다음 토픽이 자동 생성되어야 합니다:

/left_motor/motor_velocity_command
/left_motor/motor_position
/right_motor/motor_velocity_command
/right_motor/motor_position

이 토픽을 rqt_graph 또는 ros2 topic list로 확인하고
이전에 만든 diff_drive_kinematics_node 와 odom_publisher_node 와 연결하면 됩니다.
🔄 전체 구성 흐름

[cmd_vel] ──▶ [diff_drive_kinematics_node] ──▶
              /left_motor/motor_velocity_command
              /right_motor/motor_velocity_command

[ros2_canopen] ──▶ /left_motor/motor_position
                   /right_motor/motor_position ──▶ [odom_publisher_node] ──▶ /odom, /tf

✅ 도움이 필요할 수 있는 부분

    .eds 파일 정리 및 node_id 설정 방법

    CAN 장치가 slcan, socketcan인지 여부

    SDO vs PDO 설정

필요하시면 left_motor.eds와 실제 하드웨어 설정 방법도 도와드릴 수 있어요!

====================

✅ 전체 구성 요약

    canopen_master_node: CANopen 마스터 역할을 수행.

    Cia402Device: 각 모터를 제어할 수 있는 노드 타입.

    각 모터는 별도의 YAML 설정을 가짐 (EDS 포함)

📂 디렉토리 구조 예시

my_robot_canopen_ws/
├── config/
│   ├── roboteq_master.yaml
│   ├── left_motor.yaml
│   ├── right_motor.yaml
│   ├── roboteq.eds
├── launch/
│   └── bringup.launch.py

🧩 1. roboteq_master.yaml (마스터 설정)

master:
  interface_name: "can0"  # socketcan 인터페이스 이름
  node_id: 1
  dcf: false
  driver: "canopen_master_driver::CanopenMaster"

🧩 2. left_motor.yaml (왼쪽 모터 설정)

node_id: 2
driver: "ros2_canopen::Cia402Device"
eds: "config/roboteq.eds"

cia402:
  motor_velocity_command_topic: "motor_velocity_command"
  motor_position_state_topic: "motor_position"

🧩 3. right_motor.yaml (오른쪽 모터 설정)

node_id: 3
driver: "ros2_canopen::Cia402Device"
eds: "config/roboteq.eds"

cia402:
  motor_velocity_command_topic: "motor_velocity_command"
  motor_position_state_topic: "motor_position"

🧩 4. bringup.launch.py (런치 파일)

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # CANopen Master 노드
        Node(
            package='canopen_master_driver',
            executable='canopen_master_node',
            name='canopen_master',
            output='screen',
            parameters=['config/roboteq_master.yaml']
        ),

        # 왼쪽 모터
        Node(
            package='ros2_canopen',
            executable='canopen_node',
            name='left_motor',
            namespace='left_motor',
            output='screen',
            parameters=['config/left_motor.yaml'],
            remappings=[
                ('motor_velocity_command', '/left_motor/motor_velocity_command'),
                ('motor_position', '/left_motor/motor_position')
            ]
        ),

        # 오른쪽 모터
        Node(
            package='ros2_canopen',
            executable='canopen_node',
            name='right_motor',
            namespace='right_motor',
            output='screen',
            parameters=['config/right_motor.yaml'],
            remappings=[
                ('motor_velocity_command', '/right_motor/motor_velocity_command'),
                ('motor_position', '/right_motor/motor_position')
            ]
        )
    ])

📄 EDS 파일 예시 (roboteq.eds)

Roboteq에서 제공한 EDS 파일을 사용하거나 아래처럼 최소 구성으로 테스트용 제작도 가능:

[Device]
DeviceName=Roboteq Motor Controller
VendorName=Roboteq Inc.
ProductNumber=0
RevisionNumber=0x0001

[1]
ParameterName=Controlword
ObjectType=0x7
DataType=0x0005
AccessType=rw
DefaultValue=0

[6040]
ParameterName=Controlword
DataType=0x0006
AccessType=rw

[6041]
ParameterName=Statusword
DataType=0x0006
AccessType=r

[6060]
ParameterName=Mode of operation
DataType=0x0005
AccessType=rw

[6061]
ParameterName=Mode of operation display
DataType=0x0005
AccessType=r

[6064]
ParameterName=Position actual value
DataType=0x0008
AccessType=r

[606C]
ParameterName=Velocity actual value
DataType=0x0008
AccessType=r

[60FF]
ParameterName=Target velocity
DataType=0x0008
AccessType=rw

※ 실제 EDS는 Roboteq에서 제공한 것을 사용하는 것을 추천합니다.
📌 주의할 점

    can0은 sudo ip link set can0 up type can bitrate 500000 명령으로 활성화 필요

    socketcan 드라이버를 커널에서 지원해야 합니다

    Cia402Device는 DS402 프로파일을 따라야 하며, 해당 ODrive/Roboteq가 DS402를 완전 지원하는지 확인해



1. 엔코더 정보 수신 (로봇 위치·속도 계산용)

    드라이버가 퍼블리시하는 토픽:
    ~/joint_states
    예) /left_motor/joint_states, /right_motor/joint_states

    메시지 타입:
    sensor_msgs/msg/JointState

    내용:
    여기 position (휠 회전량, 엔코더 값 변환), velocity (속도), effort (토크, 보통 빈 값) 정보를 담고 있습니다.

    즉, 엔코더 값은 joint_states 토픽을 구독해서 받으면 됩니다.

2. cmd_vel (속도 명령) 변환 후 전송 방법

    드라이버는 토픽 구독으로 속도 명령을 받지 않고, 서비스 호출을 통해 명령을 받습니다!

    사용해야 할 서비스 이름 및 타입:
    /<motor_name>/target (예: /left_motor/target)
    서비스 타입: canopen_interfaces/srv/COTargetDouble

    서비스 요청 메시지 구조:

    float64 target

    따라서 cmd_vel에서 좌/우 휠 속도로 변환한 후,
    각각 아래처럼 서비스 호출해야 합니다:

auto client_left = this->create_client<canopen_interfaces::srv::COTargetDouble>("/left_motor/target");
auto client_right = this->create_client<canopen_interfaces::srv::COTargetDouble>("/right_motor/target");

auto req_left = std::make_shared<canopen_interfaces::srv::COTargetDouble::Request>();
req_left->target = left_wheel_velocity;

auto req_right = std::make_shared<canopen_interfaces::srv::COTargetDouble::Request>();
req_right->target = right_wheel_velocity;

client_left->async_send_request(req_left);
client_right->async_send_request(req_right);

3. 요약
역할	토픽 / 서비스 이름	메시지 타입	비고
엔코더 데이터	/left_motor/joint_states	sensor_msgs/msg/JointState	좌/우 휠 엔코더 상태 수신
속도 명령	/left_motor/target	canopen_interfaces/srv/COTargetDouble	서비스 호출로 목표 속도 설정

필요하면, cmd_vel 을 받아서 좌/우 휠 속도로 변환 후 위 서비스 호출하는 예제 C++ 코드도 제공해 드릴게요.
필요하시면 알려주세요!


=====

✅ Ubuntu 22.04 + ROS 2 Humble + SocketCAN 기반 커스텀 드라이버에 필요한 구성요소
1. 시스템 라이브러리 (필수)

sudo apt update
sudo apt install can-utils libsocketcan-dev

패키지명	설명
can-utils	candump, cansend 등 CAN 디버깅 툴
libsocketcan-dev	C언어 기반 SocketCAN 라이브러리 (필요시 사용)

    대부분은 Linux Socket API (socket(), bind(), read(), write())로 처리 가능합니다.
    libsocketcan은 bitrate 설정, 인터페이스 관리 등을 쉽게 도와주는 보조용입니다.

2. ROS 2 패키지

sudo apt install ros-humble-rclcpp ros-humble-std-msgs

패키지명	설명
rclcpp	ROS 2 C++ 노드 구현용
std_msgs	ROS 2 기본 메시지 사용
(선택) can_msgs	필요시 can_msgs::Frame 사용 가능 (직접 커스텀해도 됨)

    can_msgs는 꼭 필요하지는 않습니다. 직접 CAN frame 구조체를 정의해도 됩니다.

✅ 예제용 C++ 코드 구현에 필요한 요소

    SocketCAN 코드 직접 구현 (아래처럼)

int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
struct ifreq ifr;
strcpy(ifr.ifr_name, "can0");
ioctl(s, SIOCGIFINDEX, &ifr);

struct sockaddr_can addr;
addr.can_family = AF_CAN;
addr.can_ifindex = ifr.ifr_ifindex;
bind(s, (struct sockaddr *)&addr, sizeof(addr));

// 수신 대기 예시
struct can_frame frame;
read(s, &frame, sizeof(struct can_frame));

    위 코드를 기반으로 ROS 2 노드에서 수신한 CAN 데이터를 pub/sub로 전달

✅ 전체 요약: 필요한 패키지 목록
분류	패키지명	설명
🧩 시스템 라이브러리	can-utils	candump 등 테스트
	libsocketcan-dev	socketcan 인터페이스 보조
🧩 ROS2	ros-humble-rclcpp	ROS 노드 구현
	ros-humble-std-msgs	메시지 전송
(선택)	ros-humble-can-msgs	CAN frame을 표준 메시지로 쓸 경우
필요시 추천하는 구조

motor_driver_node/
├── src/
│   └── motor_driver_node.cpp  ← socket() + ROS2 rclcpp
├── CMakeLists.txt
└── package.xml

필요하시면 CMake 예제, 실제 can_frame 송수신 구현 예제도 바로 제공해드릴 수 있습니다.
You said:
cmake나 xml 좀
ChatGPT said:

커스텀 SocketCAN 기반 ROS 2 Humble 모터 드라이버 노드를 만들기 위한 CMakeLists.txt와 package.xml 템플릿은 아래와 같습니다.
✅ CMakeLists.txt

cmake_minimum_required(VERSION 3.5)
project(motor_driver_node)

# Default to C++14
set(CMAKE_CXX_STANDARD 14)

# Find ROS2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Optional: can_msgs (only if you're using ros-humble-can-msgs)
# find_package(can_msgs REQUIRED)

# Build the node
add_executable(${PROJECT_NAME}
  src/motor_driver_node.cpp
)

# Link system-level libraries (for socket, etc.)
target_link_libraries(${PROJECT_NAME}
  pthread
)

# Include headers
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Link with ROS2 packages
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  # can_msgs  # Optional
)

# Install
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

✅ package.xml

<?xml version="1.0"?>
<package format="3">
  <name>motor_driver_node</name>
  <version>0.1.0</version>
  <description>Custom SocketCAN motor driver node using ROS 2 Humble</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <!-- ROS 2 dependencies -->
  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>std_msgs</depend>

  <!-- Optional: Uncomment if using can_msgs -->
  <!-- <depend>can_msgs</depend> -->

  <!-- OS-level dependencies (not needed here since handled outside ROS) -->
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>

✅ 빌드 방법

cd ~/ros2_ws
colcon build --packages-select motor_driver_node
source install/setup.bash

✅ 실행 예시

ros2 run motor_driver_node motor_driver_node



libsocketcan-dev는 설치할 필요 없습니다
그 이유:

헤더에 포함된 부분을 보면:

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

이것은 리눅스 커널의 표준 SocketCAN 인터페이스를 직접 사용하겠다는 의미입니다. 즉, 다음 방식으로 직접 소켓을 생성하고 처리합니다:

int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
bind(s, (struct sockaddr *)&addr, sizeof(addr));
read(s, &frame, sizeof(struct can_frame));

➡️ 이 방식은 리눅스 커널에 내장된 SocketCAN 드라이버를 직접 사용하는 것으로, libsocketcan과는 관계가 없습니다.
🧾 참고: libsocketcan이 필요한 경우

libsocketcan-dev는 다음과 같은 보조 기능을 C 함수로 제공하는 라이브러리입니다:

    CAN 인터페이스 상태 가져오기 (can_get_state)

    인터페이스 이름으로 index 가져오기 (if_nametoindex()는 libc에도 있음)

    bitrate 설정, can up/down 자동화 등의 도우미 함수

하지만 이 기능 대부분은 다음과 같이 표준 C API 또는 ip link 명령어로 충분히 대체됩니다:

sudo ip link set can0 type can bitrate 500000
sudo ip link set up can0

✅ 결론
항목	필요 여부	이유
libsocketcan-dev	❌ 설치 필요 없음	코드가 직접 socket/syscall 방식으로 처리
can-utils	✅ 설치 추천	candump, cansend 등 디버깅 도구로 유용
📌 추가 팁

    위 헤더 파일에서는 boost::thread, yaml-cpp, ament_index_cpp 등이 사용되므로 다음 패키지는 설치되어 있어야 합니다:

sudo apt install libyaml-cpp-dev

    boost::thread는 ROS 2 빌드 환경에서 기본 포함되어 있지만, 빌드 에러가 나면 libboost-thread-dev도 설치하세요:

sudo apt install libboost-thread-dev



==

// left_vel, right_vel 값을 바퀴 모터로 전송

✅ 3. odometry_node 사용 예시

#include "kinematics.hpp"

kinematics::DifferentialDrive kinematics(0.05, 0.3);  // 동일한 파라미터로 초기화

double left_vel = measured_left_rpm * rpm_to_radps;
double right_vel = measured_right_rpm * rpm_to_radps;

double linear_x, angular_z;
kinematics.wheelToTwist(left_vel, right_vel, linear_x, angular_z);

// linear_x, angular_z 값을 odom 메시지에 활용\
\




#include "rclcpp/rclcpp.hpp"
#include "kinematics/kinematics_factory.hpp"
#include "kinematics/base_kinematics.hpp"
#include <memory>

class OdomNode : public rclcpp::Node
{
public:
    OdomNode(const std::string& drive_type)
    : Node("odom_node")
    {
        // 주행 방식에 따른 기구학 객체 생성
        kinematics_ = kinematics::KinematicsFactory::create(drive_type);
        if (!kinematics_) {
            RCLCPP_ERROR(this->get_logger(), "Invalid drive type: %s", drive_type.c_str());
            throw std::runtime_error("Invalid drive type");
        }

        // 바퀴 치수 설정 (예시)
        kinematics_->setWheelRadius(0.05);
        kinematics_->setWheelBase(0.3);
        kinematics_->setWheelSeparation(0.3);

        // 구독, 타이머 등 초기화...
    }

private:
    std::unique_ptr<kinematics::BaseKinematics> kinematics_;
    // 기타 멤버: odom 값, 구독자, 발행자 등
};


#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "kinematics/base_kinematics.hpp"

class OdomNode : public rclcpp::Node
{
public:
    OdomNode(const std::string& drive_type)
    : Node("odom_node")
    {
        kinematics_ = kinematics::KinematicsFactory::create(drive_type);
        if (!kinematics_) {
            RCLCPP_ERROR(this->get_logger(), "Invalid drive type: %s", drive_type.c_str());
            throw std::runtime_error("Invalid drive type");
        }

        kinematics_->setWheelRadius(0.05);
        kinematics_->setWheelBase(0.3);
        kinematics_->setWheelSeparation(0.3);

        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10,
            std::bind(&OdomNode::jointStateCallback, this, std::placeholders::_1));
    }

private:
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // 예: 왼쪽, 오른쪽 바퀴 속도(rad/s) 얻기 (position 또는 velocity 필드 사용 가능)
        // joint 이름에 따라 순서 맞춰 읽어야 함

        double left_wheel_vel = 0.0;
        double right_wheel_vel = 0.0;

        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (msg->name[i] == "left_wheel_joint") {
                // 속도 필드가 없으면 position 차이 등으로 계산 필요
                if (msg->velocity.size() > i)
                    left_wheel_vel = msg->velocity[i];
            }
            else if (msg->name[i] == "right_wheel_joint") {
                if (msg->velocity.size() > i)
                    right_wheel_vel = msg->velocity[i];
            }
        }

        // 벡터로 변환
        std::vector<double> wheel_velocities = {left_wheel_vel, right_wheel_vel};

        double linear_x, linear_y, angular_z;
        kinematics_->wheelVelocitiesToTwist(wheel_velocities, linear_x, linear_y, angular_z);

        RCLCPP_INFO(this->get_logger(),
            "linear_x: %.3f, linear_y: %.3f, angular_z: %.3f",
            linear_x, linear_y, angular_z);

        // 여기서 Odometry 적분, 메시지 발행 등 추가
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    std::unique_ptr<kinematics::BaseKinematics> kinematics_;
};



1. 워크스페이스 빌드

터미널에서 워크스페이스 루트 (~/inabot_ws 등)로 이동 후:

colcon build --packages-select inabot_core

빌드가 완료되면:

source install/setup.bash

(매번 터미널 열 때마다 source 하거나 ~/.bashrc에 추가하세요)
2. 런치 파일 실행

빌드 후에 다음 명령어로 런치 실행:

ros2 launch inabot_core inabot_core.launch.py
