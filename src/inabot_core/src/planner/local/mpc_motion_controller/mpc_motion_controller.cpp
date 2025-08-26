// mpc_motion_controller.cpp
#include "planner/local/mpc_motion_controller/mpc_motion_controller.hpp"

//보간 tk::spline 사용 (Cubic 보간도 가능)
#include "planner/local/mpc_motion_controller/spline.h"

#define __LOG__ false
#define __MPC_LOG__ true

  
MpcMotionController::MpcMotionController(const rclcpp::NodeOptions& options)
  : Node("mpc_motion_controller", options) 
{
    throttle_ = 0.0;
    w_ = 0.0;
    speed_ = 0.0;

    initParameters();

    // PathProvider 공유 인스턴스 생성
    path_provider_ = std::make_shared<PathProvider>(this->get_logger());

    //void LoadParams(const std::map<string, double> &params)
    _mpc.LoadParams(_mpc_params);
    #if _SET_QOS_
    auto odom_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
    #endif
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", /*odom_qos*/ 10,
      std::bind(&MpcMotionController::odometry_callback, this, std::placeholders::_1));
  
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/planned_path", 10, std::bind(&MpcMotionController::pathCallback, this, std::placeholders::_1)); //plan
 
    obstacle_sub_ = this->create_subscription<inabot_msgs::msg::ObstacleStatus>(
        "/obstacle_status", 10,
        [this](const inabot_msgs::msg::ObstacleStatus::SharedPtr msg){
            if(msg) {
                obstacle_distance_ = msg->distance;
                obstacle_stop_ = msg->stop;

                RCLCPP_INFO(this->get_logger(),
                                "[ObstacleStatus] distance: %.3f m, stop: %s",
                                obstacle_distance_,
                                obstacle_stop_ ? "true" : "false");
            }
        });

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", rclcpp::QoS(10));

    goal_reached_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/goal_reached", rclcpp::QoS(10));

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20), std::bind(&MpcMotionController::timerCallback, this)); // 100 -> 20ms
}

MpcMotionController::~MpcMotionController() {}

void MpcMotionController::initParameters()
{
    this->declare_parameter("mpc_dt", 0.066);
    this->declare_parameter("mpc_steps", 10.0);
    this->declare_parameter("mpc_ref_cte", 0.0);
    this->declare_parameter("mpc_ref_etheta", 0.0);
    this->declare_parameter("mpc_ref_velocity", 1.0);
    this->declare_parameter("mpc_weight_cte", 5000.0);
    this->declare_parameter("mpc_weight_etheta", 5000.0);
    this->declare_parameter("mpc_weight_velocity", 1.0);
    this->declare_parameter("mpc_weight_angvel", 100.0);
    this->declare_parameter("mpc_weight_accel", 50.0);
    this->declare_parameter("mpc_weight_dangvel", 10.0);
    this->declare_parameter("mpc_weight_daccel", 10.0);
    this->declare_parameter("mpc_max_angular_velocity", 3.0);
    this->declare_parameter("mpc_max_throttle", 1.0);
    this->declare_parameter("mpc_bound_value", 1000.0);
    this->declare_parameter("goal_threshold", 0.1);
    this->declare_parameter("max_speed", 0.5);
    this->declare_parameter("delay_mode", false);
  
    // 새로 추가하는 파라미터
    this->declare_parameter("interp_spacing", 0.4);
    this->declare_parameter("min_points", 4);
    this->declare_parameter("min_safe_distance", 0.2);
    this->declare_parameter("max_stop_distance", 1.0);
 
    this->get_parameter("mpc_dt", dt_);
    this->get_parameter("mpc_steps", mpc_steps_);
    this->get_parameter("mpc_ref_cte", ref_cte_);
    this->get_parameter("mpc_ref_etheta", ref_etheta_);
    this->get_parameter("mpc_ref_velocity", ref_vel_);
    this->get_parameter("mpc_weight_cte", w_cte_);
    this->get_parameter("mpc_weight_etheta", w_etheta_);
    this->get_parameter("mpc_weight_velocity", w_vel_);
    this->get_parameter("mpc_weight_angvel", w_angvel_);
    this->get_parameter("mpc_weight_accel", w_accel_);
    this->get_parameter("mpc_weight_dangvel", w_angvel_d_);
    this->get_parameter("mpc_weight_daccel", w_accel_d_);
    this->get_parameter("mpc_max_angular_velocity", max_angvel_);
    this->get_parameter("mpc_max_throttle", max_throttle_);
    this->get_parameter("mpc_bound_value", bound_value_);
    this->get_parameter("goal_threshold", goal_threshold_);
    this->get_parameter("max_speed", max_speed_);
    this->get_parameter("delay_mode", delay_mode_);

    // 새로 추가하는 파라미터 가져오기
    this->get_parameter("interp_spacing", interp_spacing_);
    this->get_parameter("min_points", min_points_);
    this->get_parameter("min_safe_distance", min_safe_distance_);
    this->get_parameter("max_stop_distance", max_stop_distance_);

    RCLCPP_INFO(this->get_logger(), "MPC parameters:");
    RCLCPP_INFO(this->get_logger(), "  mpc_dt: %.6f", dt_);
    RCLCPP_INFO(this->get_logger(), "  mpc_steps: %.1f", mpc_steps_);
    RCLCPP_INFO(this->get_logger(), "  mpc_ref_cte: %.6f", ref_cte_);
    RCLCPP_INFO(this->get_logger(), "  mpc_ref_etheta: %.6f", ref_etheta_);
    RCLCPP_INFO(this->get_logger(), "  mpc_ref_velocity: %.6f", ref_vel_);
    RCLCPP_INFO(this->get_logger(), "  mpc_weight_cte: %.1f", w_cte_);
    RCLCPP_INFO(this->get_logger(), "  mpc_weight_etheta: %.1f", w_etheta_);
    RCLCPP_INFO(this->get_logger(), "  mpc_weight_velocity: %.6f", w_vel_);
    RCLCPP_INFO(this->get_logger(), "  mpc_weight_angvel: %.1f", w_angvel_);
    RCLCPP_INFO(this->get_logger(), "  mpc_weight_accel: %.1f", w_accel_);
    RCLCPP_INFO(this->get_logger(), "  mpc_weight_dangvel: %.1f", w_angvel_d_);
    RCLCPP_INFO(this->get_logger(), "  mpc_weight_daccel: %.1f", w_accel_d_);
    RCLCPP_INFO(this->get_logger(), "  mpc_max_angular_velocity: %.3f", max_angvel_);
    RCLCPP_INFO(this->get_logger(), "  mpc_max_throttle: %.3f", max_throttle_);
    RCLCPP_INFO(this->get_logger(), "  mpc_bound_value: %.1f", bound_value_);
    RCLCPP_INFO(this->get_logger(), "  goal_threshold: %.3f", goal_threshold_);
    RCLCPP_INFO(this->get_logger(), "  max_speed: %.3f", max_speed_);
    RCLCPP_INFO(this->get_logger(), "  delay_mode: %s", delay_mode_ ? "true" : "false");
    // 새로 추가한 파라미터 로그 출력
    RCLCPP_INFO(this->get_logger(), "  interp_spacing_: %.3f", interp_spacing_);
    RCLCPP_INFO(this->get_logger(), "  min_points_: %d", min_points_);
    
    // Init parameter map for MPC
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
}

void MpcMotionController::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_odom_ = *msg;

    const auto& pos = msg->pose.pose.position;
    const auto& ori = msg->pose.pose.orientation;
    const auto& vel = msg->twist.twist.linear;

    double yaw = tf2::getYaw(ori);
#if __LOG__
    RCLCPP_DEBUG(this->get_logger(), "[Odometry] Pos: (%.2f, %.2f), Yaw: %.2f rad, Linear Vel: %.2f m/s",
        pos.x, pos.y, yaw, vel.x);
#endif
}

bool MpcMotionController::isOdomValid() const
{
    const auto& pos = current_odom_.pose.pose.position;
    const auto& ori = current_odom_.pose.pose.orientation;

    return !(pos.x == 0.0 && pos.y == 0.0 && pos.z == 0.0 &&
             ori.x == 0.0 && ori.y == 0.0 &&
             ori.z == 0.0 && ori.w == 0.0);
}

// ============================================================
// 1Path Callback
// ============================================================
void MpcMotionController::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
    if (!isOdomValid()) {
        RCLCPP_WARN(this->get_logger(), "[Odom Check] Invalid odometry data.");
        return;
    }

    if (msg->poses.empty()) {
        RCLCPP_WARN(this->get_logger(), "Received empty path.");
        return;
    }

    const auto& current_pose = current_odom_.pose.pose;

    // 목표 도착 체크 (한 점일 경우와 n개 경로 공통)
    if (path_provider_->isWithinGoalThreshold(msg->poses.back().pose, current_pose, goal_threshold_)) {
        publishGoalReached(true);
        path_provider_->clearPath();
        return;
    }

    // 단일 목표 점일 경우
    if (msg->poses.size() == 1) {
      global_plan_ = path_provider_->generateInterpolatedPathForSinglePoint(
        msg->poses.front(),
        current_pose,
        interp_spacing_,
        min_points_
      );
    }
    else 
    {
      // 여러 점일 경우 (FMS 전체 경로)
      global_plan_ = path_provider_->generateSplinePathFromMultiplePoints(
        msg->poses,         // FMS에서 받은 모든 포인트
        current_pose,       // 현재 위치
        interp_spacing_,    // 보간 간격 (예: 0.4m)
        min_points_         // 최소 점 개수 (예: 4)
      );
    }

    // PathProvider 내부에 업데이트
    path_provider_->updatePath(global_plan_);

#if __LOG__
    path_provider_->logGlobalPlanInfo(global_plan_);
#endif
}

// ============================================================
// Timer Callback
// ============================================================
void MpcMotionController::timerCallback()
{
    if (!isOdomValid()) {
        RCLCPP_WARN(this->get_logger(), "No odometry received.");
        return;
    }

    if (path_provider_->size() < 1) {
        RCLCPP_WARN(this->get_logger(), "No planned path available.");
        return;
    }

    // MPC를 이용해 현재 경로 기준 최적 제어 입력 계산
    auto cmd_vel = computeVelocityCommands();

    // 장애물 거리 기반 감속 처리
    if (obstacle_stop_ && obstacle_distance_ < max_stop_distance_) {
        double decel_ratio = (obstacle_distance_ - min_safe_distance_) /
                             (max_stop_distance_ - min_safe_distance_);
        decel_ratio = std::clamp(decel_ratio, 0.0, 1.0);

        double original_linear = cmd_vel.twist.linear.x;
        cmd_vel.twist.linear.x *= decel_ratio;

        if (decel_ratio == 0.0) cmd_vel.twist.angular.z = 0.0;

        RCLCPP_INFO(this->get_logger(),
            "[Obstacle] distance: %.2f m, decel_ratio: %.2f, linear: %.2f -> %.2f, angular: %.2f",
            obstacle_distance_, decel_ratio, original_linear, cmd_vel.twist.linear.x, cmd_vel.twist.angular.z);
    }

    cmd_vel_pub_->publish(cmd_vel.twist);
}

// ============================================================
// Compute Velocity Commands (MPC)
// ============================================================
geometry_msgs::msg::TwistStamped MpcMotionController::computeVelocityCommands()
{
    geometry_msgs::msg::TwistStamped cmd_vel;
    const double dt = dt_;
    const double w = w_;               // steering -> w
    const double throttle = throttle_; // accel: >0; brake: <0

    // 경로 확인
    const auto& path = path_provider_->getPath();

    // 0. 경로 유무 체크
    if (path.size() == 0) {
        cmd_vel.twist.linear.x = 0.0;
        cmd_vel.twist.angular.z = 0.0;
        return cmd_vel;
    }

    if (path.size() < 4) { // MPC 최소 점 수 4개
        RCLCPP_WARN(get_logger(), "Not enough path points for MPC.");
        cmd_vel.twist.linear.x = 0.0;
        cmd_vel.twist.angular.z = 0.0;
        return cmd_vel;
    }

    // 현재 위치와 자세
    const auto& pose = current_odom_.pose.pose;
    double cur_x = pose.position.x;
    double cur_y = pose.position.y;

    tf2::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // 경로를 차량 좌표계로 변환
    std::vector<double> ptsx, ptsy;
    for (const auto& wp : path) {
        double dx = wp.pose.position.x - cur_x;
        double dy = wp.pose.position.y - cur_y;
        double local_x = dx * cos(-yaw) - dy * sin(-yaw);
        double local_y = dx * sin(-yaw) + dy * cos(-yaw);
        ptsx.push_back(local_x);
        ptsy.push_back(local_y);
    }

#if __MPC_LOG__
    RCLCPP_DEBUG(get_logger(), "[MPC] Current pose - x: %.3f, y: %.3f, yaw: %.3f", cur_x, cur_y, yaw);
#endif

    //경로 포인트가 4개 미만이면 다항식 피팅과 MPC 계산이 신뢰x
    if (ptsx.size() < 4) {
        RCLCPP_WARN(get_logger(), "Not enough path points for MPC.");
        cmd_vel.twist.linear.x = 0.0;
        cmd_vel.twist.angular.z = 0.0;
        return cmd_vel;
    }

    // 3차 다항식 피팅
    Eigen::VectorXd ptsx_eig = Eigen::Map<Eigen::VectorXd>(ptsx.data(), ptsx.size());
    Eigen::VectorXd ptsy_eig = Eigen::Map<Eigen::VectorXd>(ptsy.data(), ptsy.size());
    Eigen::VectorXd coeffs = polyfit(ptsx_eig, ptsy_eig, 3);

#if __MPC_LOG__
    RCLCPP_DEBUG(get_logger(), "[MPC] Polynomial coefficients: %.5f, %.5f, %.5f, %.5f",
                 coeffs[0], coeffs[1], coeffs[2], coeffs[3]);
#endif

    //  1. cte (Cross Track Error, 횡방향 오차)
    //      polyeval(coeffs, 0.0) 은 다항식 경로에서 x=0 위치에서의 y값을 계산
    //      여기서 x=0은 차량 기준 좌표계에서 차량 위치를 의미
    //      차량의 실제 y좌표는 0 (로컬 좌표에서), 따라서 다항식 y값과 실제 위치 y(=0) 차이가 횡방향 오차
    //      즉, cte = f(0) - 0 = f(0) 인 거죠.
    //  2. etheta (Heading Error, 주행 방향 오차)
    //      impThetaError 함수는 차량의 현재 heading(yaw)과 경로의 곡선 방향 사이의 각도 차이를 계산
    //      경로 방향은 다항식의 미분값, 즉 기울기 f'(x)
    //      함수 내부적으로 차량과 경로 사이 각도 차를 구하는 로직
    //  3. epsi (이상적인 heading error)
    //      주석 처리된 코드 double epsi = 0.0 - std::atan(coeffs[1]); 은 일반적으로 많이 쓰는 공식
    //      여기서 coeffs[1] 은 다항식 1차 항 계수(기울기)
    //      atan(coeffs[1]) 은 경로의 기울기 각도를 구하는 식
    //      차량 기준 좌표계에서 차량 heading이 0일 때,
    //      epsi = 0 - atan(f'(0)) 로 차량 방향과 경로 방향 차이
    //      따라서 epsi 를 etheta 대신 사용 

    double v = current_odom_.twist.twist.linear.x;
    double cte = polyeval(coeffs, 0.0); // cte = f(0) - 0 = f(0) <- x = 0 기준, x=0에서의 오차
    double etheta = impThetaError(yaw, coeffs, ptsx.size(), 0.3);
    //double epsi = 0.0 - std::atan(coeffs[1]); // f'(0) 기준
#if __MPC_LOG__
    RCLCPP_DEBUG(get_logger(), "[MPC] Vehicle state - v: %.3f, CTE: %.3f, eTheta: %.3f", v, cte, etheta);
#endif
    // 차량 상태 벡터
    Eigen::VectorXd state(6);

    /**
    | 인덱스 | 변수명     | 의미                     | 현재 코드에서의 값           |
    | --- | ------- | ---------------------- | -------------------- |
    | 0   | x       | 차량의 x 위치 (로컬 차량 좌표계에서) | 0 (로컬 좌표계 기준으로 항상 0) |
    | 1   | y       | 차량의 y 위치               | 0 (로컬 좌표계 기준으로 항상 0) |
    | 2   | ψ (yaw) | 차량의 헤딩 (yaw 각도)        | 0 (로컬 좌표계 기준으로 항상 0) |
    | 3   | v       | 현재 차량 속도               | 실제 속도 (현재 odom으로부터)  |
    | 4   | cte     | 크로스 트랙 에러 (경로와의 거리 오차) | 다항식 평가한 값, x=0 기준    |
    | 5   | etheta  | 차량 진행 방향과 목표 궤적 기울기 차이 | yaw 오차 (자세 오차)       |
    */

    if (delay_mode_)
    {
        // Kinematic model is used to predict vehicle state at the actual moment of
        // control (current time + delay dt)
        const double px_act = v * dt;
        const double py_act = 0;
        const double theta_act = w * dt; //(steering) theta_act = v * steering * dt / Lf;
        const double v_act = v + throttle * dt; // v = v + a * dt

        const double cte_act = cte + v * sin(etheta) * dt;
        const double etheta_act = etheta - theta_act;

        state << px_act, py_act, theta_act, v_act, cte_act, etheta_act;
    }
    else
    {
        state << 0, 0, 0, v, cte, etheta;
    }  
 
    // MPC 최적화
    //주어진 상태에서 차량이 앞으로 어떻게 움직일지 예측 모델을 기반으로차량의 물리적 동역학을 반영해
    //경로와 최대한 일치하면서도 제약조건(속도 제한, 최대 가속, 조향 각도 등)을 지키는 최적의 제어 명령 
    //(가속도, 조향각 속도 등)을 계산해 반환

    auto start_time = rclcpp::Clock().now();
    std::vector<double> mpc_results = _mpc.Solve(state, coeffs);
    auto end_time = rclcpp::Clock().now();

    if (mpc_results.size() < 2) {
        RCLCPP_ERROR(get_logger(), "MPC result invalid.");
        cmd_vel.twist.linear.x = 0.0;
        cmd_vel.twist.angular.z = 0.0;
        return cmd_vel;
    }
#if __MPC_LOG__
    RCLCPP_INFO(get_logger(), "MPC Solve duration: %.6f sec", (end_time - start_time).seconds());
#endif
    
    // MPC 결과 처리
    w_ = mpc_results[0];        // 각속도(rad/s)
    throttle_ = mpc_results[1]; // 가속도(m/s^2)
    speed_ = std::clamp(v + throttle_ * dt_, 0.0, max_speed_); ////max_speed_ = 0.5 m/s

    cmd_vel.header = current_odom_.header;
    cmd_vel.twist.linear.x = speed_;
    cmd_vel.twist.angular.z = w_;

#if __MPC_LOG__
    RCLCPP_INFO(get_logger(),
        "[MPC] Output - Linear Velocity: %.3f m/s, Angular Velocity: %.3f rad/s",
        speed_, w_);
#endif

    return cmd_vel;
}

// polyeval 함수
//경로에서의 현재 차량 위치 오차(CTE, cross track error)**를 계산
double MpcMotionController::polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * std::pow(x, i);
  }
  return result;
}

// polyfit 함수
//이 함수는 경로 점 집합에 대해 최적의(최소 제곱 오차) 다항식 근사식을 찾는 데 사용
//이렇게 구한 다항식 계수들은 MPC에서 경로 추종 시 참고할 수 있도록 경로를 수식으로 표현하는 데 활용
Eigen::VectorXd MpcMotionController::polyfit(Eigen::VectorXd xvals,
                                       Eigen::VectorXd yvals, int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++)
    A(i, 0) = 1.0;

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++)
      A(j, i + 1) = A(j, i) * xvals(j);
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

//현재 차량의 방향(heading)과 경로의 전반적인 진행 방향 간의 각도 오차(heading error)를 계산
double MpcMotionController::impThetaError(double theta, const Eigen::VectorXd& coeffs,
                                         int sample_size, int sample_ratio) {
  double etheta = atan(coeffs[1]);
  double gx = 0;
  double gy = 0;

  const auto& path = path_provider_->getPath();

  for (int i = 1; i < sample_size * sample_ratio && i < static_cast<int>(path.size()); i++) {
    gx += path[i].pose.position.x - path[i - 1].pose.position.x;
    gy += path[i].pose.position.y - path[i - 1].pose.position.y;
  }

  double temp_theta = theta;
  double traj_deg = atan2(gy, gx);
  double PI = 3.141592;

  if (temp_theta <= -PI + traj_deg)
    temp_theta = temp_theta + 2 * PI;

  if (gx && gy && temp_theta - traj_deg < 1.8 * PI)
    etheta = temp_theta - traj_deg;
  else
    etheta = 0;

#if __LOG__
  std::cout << "etheta: " << etheta << ", atan2(gy,gx): " << atan2(gy, gx)
            << ", temp_theta:" << traj_deg << std::endl;
#endif
  return etheta;
}

void MpcMotionController::publishGoalReached(bool reached) {
  std_msgs::msg::Bool msg;
  msg.data = reached;
  goal_reached_pub_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "Goal reached!");
}
