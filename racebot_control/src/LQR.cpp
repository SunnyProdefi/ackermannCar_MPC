// LQR.cpp
#include "LQR.h"
#include <algorithm>
#include <cmath>
#include <cmath>  // 用于计算数学函数
#include <cmath>
#include <fstream>
#include <iostream>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <numeric>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <stdexcept>
#include <string>
#include <vector>
#include <visualization_msgs/Marker.h>

double x_gazebo = 0.0, y_gazebo = 0.0, yaw_gazebo = 0.0, v_gazebo = 0.0;

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  // 获取位置信息
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;

  // 获取方向（四元数）并计算偏航角（yaw）
  double qx = msg->pose.pose.orientation.x;
  double qy = msg->pose.pose.orientation.y;
  double qz = msg->pose.pose.orientation.z;
  double qw = msg->pose.pose.orientation.w;
  double yaw =
      atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));

  // 获取线速度并计算车辆速度的模
  double linear_speed_x = msg->twist.twist.linear.x;
  double linear_speed_y = msg->twist.twist.linear.y;
  double linear_speed_z = msg->twist.twist.linear.z;
  double speed = sqrt(pow(linear_speed_x, 2) + pow(linear_speed_y, 2) +
                      pow(linear_speed_z, 2));

  // 打印x, y, speed, yaw
  //   ROS_INFO_STREAM("Position -> x: " << std::fixed << std::setprecision(2)
  //   << x
  //                                     << ", y: " << y);
  //   ROS_INFO_STREAM("Speed: " << std::fixed << std::setprecision(2) <<
  //   speed); ROS_INFO_STREAM("Yaw: " << std::fixed << std::setprecision(2) <<
  //   yaw);
  x_gazebo = x;
  y_gazebo = y;
  yaw_gazebo = yaw;
  v_gazebo = speed;
}

VehicleState::VehicleState(double x, double y, double yaw, double v, Gear gear)
    : x(x),
      y(y),
      yaw(yaw),
      v(v),
      e_cg(0.0),
      theta_e(0.0),
      steer(0.0),
      gear(gear) {}

void VehicleState::updateVehicleState(double delta, double a, double e_cg,
                                      double theta_e, Gear gear) {
  double wheelbase = l_r + l_f;
  regulateInput(delta, a);

  this->gear = gear;
  this->steer = delta;
  // this->x += this->v * std::cos(this->yaw) * ts;
  // this->y += this->v * std::sin(this->yaw) * ts;
  // this->yaw += this->v / wheelbase * std::tan(delta) * ts;
  this->e_cg = e_cg;
  this->theta_e = theta_e;

  if (gear == Gear::DRIVE) {
    this->v += a * ts;
  } else {
    this->v -= a * ts;  // Note the negation for reverse gear
  }

  this->v = regulateOutput(this->v);
}

void VehicleState::regulateInput(double &delta, double &a) {
  delta = std::max(delta, -max_steer_angle);
  delta = std::min(delta, max_steer_angle);

  a = std::max(a, -max_acceleration);
  a = std::min(a, max_acceleration);
}

double VehicleState::regulateOutput(double v) {
  return std::max(std::min(v, max_speed), -max_speed);
}

TrajectoryAnalyzer::TrajectoryAnalyzer(const std::vector<double> &x,
                                       const std::vector<double> &y,
                                       const std::vector<double> &yaw,
                                       const std::vector<double> &k)
    : x_(x), y_(y), yaw_(yaw), k_(k), ind_old(0), ind_end(x.size()) {}

double theta_e = 0.0;
double e_cg = 0.0;
double yaw_ref = 0.0;
double k_ref = 0.0;

void TrajectoryAnalyzer::toTrajectoryFrame(const VehicleState &vehicle_state) {
  double x_cg = vehicle_state.x;
  double y_cg = vehicle_state.y;
  double yaw = vehicle_state.yaw;

  double min_distance = std::numeric_limits<double>::max();
  size_t nearest_index = ind_old;

  // Calculate nearest point
  for (size_t i = ind_old; i < ind_end; ++i) {
    double dx = x_cg - x_[i];
    double dy = y_cg - y_[i];
    double distance = std::sqrt(dx * dx + dy * dy);
    if (distance < min_distance) {
      min_distance = distance;
      nearest_index = i;
    }
  }
  double dx = x_cg - x_[nearest_index];
  double dy = y_cg - y_[nearest_index];
  Eigen::Vector2d vec_axle_rot_90(std::cos(yaw + PI / 2.0),
                                  std::sin(yaw + PI / 2.0));
  Eigen::Vector2d vec_path_to_cg(dx, dy);
  // Calculate lateral error
  e_cg = vec_axle_rot_90.transpose().dot(vec_path_to_cg);
  e_cg = (e_cg > 0.0) ? min_distance : -min_distance;
  // Calculate yaw error
  yaw_ref = yaw_[nearest_index];
  theta_e = normalizeAngle(yaw - yaw_ref);
  // Update index and curvature reference
  ind_old = nearest_index;
  k_ref = k_[nearest_index];
}

double normalizeAngle(double angle) {
  while (angle > PI) angle -= 2 * PI;
  while (angle < -PI) angle += 2 * PI;
  return angle;
}

LatController::LatController() {}

std::tuple<double, double, double> LatController::computeControlCommand(
    const VehicleState &vehicle_state,
    const TrajectoryAnalyzer &trajectory_analyzer) {
  // Ensure trajectory_analyzer is not const if necessary
  TrajectoryAnalyzer &oTrajectoryFrame =
      const_cast<TrajectoryAnalyzer &>(trajectory_analyzer);
  double theta_e_old, e_cg_old;
  e_cg_old = vehicle_state.e_cg;
  theta_e_old = vehicle_state.theta_e;
  oTrajectoryFrame.toTrajectoryFrame(vehicle_state);

  auto [matrix_ad_, matrix_bd_] = updateMatrix(vehicle_state);

  double eps = 0.01;         // Example value, define as needed
  int max_iteration = 1000;  // Example value, define as needed

  Eigen::MatrixXd matrix_q_(4, 4);

  matrix_q_.setZero();

  matrix_q_.diagonal() << 0.5, 0.0, 1.0, 0.0;

  Eigen::MatrixXd matrix_r_(1, 1);
  matrix_r_ << 1.0;

  // Solve LQR problem with defined variables
  auto matrix_k_ = solveLQRProblem(matrix_ad_, matrix_bd_, matrix_q_, matrix_r_,
                                   eps, max_iteration);
  Eigen::MatrixXd matrix_state_(state_size, 1);
  matrix_state_ << e_cg, (e_cg - e_cg_old) / ts, theta_e,
      (theta_e - theta_e_old) / ts;

  Eigen::MatrixXd result = matrix_k_ * matrix_state_;
  double steer_angle_feedback = -result(0, 0);
  double steer_angle_feedforward = computeFeedForward(k_ref);
  double steer_angle = steer_angle_feedback + steer_angle_feedforward;

  return std::make_tuple(steer_angle, theta_e, e_cg);
}

double LatController::computeFeedForward(double ref_curvature) {
  double wheelbase = l_f + l_r;
  return wheelbase * ref_curvature;
}

Eigen::MatrixXd LatController::solveLQRProblem(const Eigen::MatrixXd &A,
                                               const Eigen::MatrixXd &B,
                                               const Eigen::MatrixXd &Q,
                                               const Eigen::MatrixXd &R,
                                               double tolerance,
                                               int max_num_iteration) {
  // Assert to check matrix dimensions
  assert(A.rows() == A.cols() && B.rows() == A.rows() && Q.rows() == Q.cols() &&
         Q.rows() == A.cols() && R.rows() == R.cols() && R.rows() == B.cols());
  Eigen::MatrixXd P = Q;
  Eigen::MatrixXd K;
  Eigen::MatrixXd M = Eigen::MatrixXd::Zero(Q.rows(), R.cols());
  int num_iteration = 0;
  double diff = std::numeric_limits<double>::infinity();

  while (num_iteration < max_num_iteration && diff > tolerance) {
    Eigen::MatrixXd K_term = R + B.transpose() * P * B;
    Eigen::MatrixXd inverse_K_term = K_term.inverse();

    Eigen::MatrixXd APB_plus_M = A.transpose() * P * B + M;
    Eigen::MatrixXd BPA_plus_MT = B.transpose() * P * A + M.transpose();

    // Ensure dimensions are correct by checking intermediary dimensions

    Eigen::MatrixXd middle_term = APB_plus_M * inverse_K_term * BPA_plus_MT;

    Eigen::MatrixXd P_next = A.transpose() * P * A - middle_term + Q;

    // Check final dimensions

    diff = (P_next - P).cwiseAbs().maxCoeff();
    P = P_next;
    num_iteration++;
  }

  if (num_iteration >= max_num_iteration) {
    std::cout << "LQR solver cannot converge to a solution, last consecutive "
                 "result diff is: "
              << diff << std::endl;
  }

  K = (B.transpose() * P * B + R).inverse() *
      (B.transpose() * P * A + M.transpose());

  return K;
}

std::tuple<Eigen::MatrixXd, Eigen::MatrixXd> LatController::updateMatrix(
    const VehicleState &vehicle_state) {
  double v = vehicle_state.v;
  double wheelbase = l_f + l_r;
  Eigen::MatrixXd matrix_ad(state_size, state_size);
  Eigen::MatrixXd matrix_bd(state_size, 1);

  matrix_ad << 1.0, ts, 0, 0, 0, 0, v, 0, 0, 0, 1.0, ts, 0, 0, 0, 0;
  matrix_bd << 0, 0, 0, v / wheelbase;

  return {matrix_ad, matrix_bd};
}

double LonController::computeControlCommand(double target_speed,
                                            const VehicleState &vehicle_state,
                                            double dist) {
  double direct = vehicle_state.gear == Gear::DRIVE ? 1.0 : -1.0;
  double a = 0.3 * (target_speed - direct * vehicle_state.v);

  if (dist < 10.0) {
    if (vehicle_state.v > 2.0) {
      a = -3.0;  // Decelerate if closing in on the target and moving too fast
    } else if (vehicle_state.v < -2) {
      a = -1.0;  // Adjust for negative velocities in reverse gear
    }
  }

  return a;
}

std::tuple<std::vector<double>, std::vector<double>, std::vector<double>,
           std::vector<double>, std::vector<double>>
PathGenerator::generateSineWavePath(double amplitude, double frequency,
                                    double length, double step) {
  std::vector<double> path_x, path_y, yaw, curvature, direction;
  for (double x = 0; x <= length; x += step) {
    double y = amplitude * std::sin(frequency * x);
    double yaw_angle =
        std::atan(amplitude * frequency * std::cos(frequency * x));
    double radius = std::fabs(
        1 / (amplitude * frequency * frequency * std::cos(frequency * x) +
             1e-10));  // Avoid division by zero
    double curv = 1 / radius;

    path_x.push_back(x);
    path_y.push_back(y);
    yaw.push_back(yaw_angle);
    curvature.push_back(curv);
    direction.push_back(1.0);  // Always moving forward
  }
  // 写入CSV文件
  std::ofstream file("path.csv");
  for (size_t i = 0; i < path_x.size(); ++i) {
    file << path_x[i] << "," << path_y[i] << "," << yaw[i] << ","
         << curvature[i] << "," << direction[i] << std::endl;
  }
  return {path_x, path_y, yaw, curvature, direction};
}

void publish_trajectory(const std::vector<double> &cx,
                        const std::vector<double> &cy,
                        ros::Publisher &marker_pub) {
  visualization_msgs::Marker points;  // 创建一个Marker消息，用于显示点
  points.header.frame_id = "odom";         // 设置frame_id为"odom"
  points.header.stamp = ros::Time::now();  // 设置时间戳为当前时间
  points.ns = "trajectory";  // 设置这组Marker的命名空间为"trajectory"
  points.action = visualization_msgs::Marker::ADD;  // 设置动作为添加（ADD）
  points.pose.orientation.w = 1.0;  // 设置方向的四元数，这里表示无旋转
  points.id = 0;  // 设置这个Marker的ID，可以用于识别和修改
  points.type = visualization_msgs::Marker::POINTS;  // 设置Marker类型为点
  points.scale.x = 0.01;  // 设置点的X方向大小
  points.scale.y = 0.01;  // 设置点的Y方向大小
  points.color.g = 1.0f;  // 设置颜色为绿色
  points.color.a = 1.0;   // 设置颜色的不透明度为完全不透明

  // 填充轨迹点
  for (size_t i = 0; i < cx.size(); ++i) {
    geometry_msgs::Point p;  // 创建一个点
    p.x = cx[i];             // 设置点的X坐标
    p.y = cy[i];             // 设置点的Y坐标
    p.z = 0;  // 设置点的Z坐标为0，因为这是一个2D轨迹
    points.points.push_back(p);  // 将点添加到Marker的点列表中
  }

  marker_pub.publish(points);  // 发布Marker消息
}

int main(int argc, char **argv) {
  double amplitude = 1.0;
  double frequency = 0.5;
  double length = 3.0;
  double step = 0.01;

  ros::init(argc, argv, "lqr_node");
  ros::NodeHandle nh;

  ROS_INFO("Configuration loading complete. Node is now spinning.");

  // 发布轨迹
  ros::Publisher marker_pub =
      nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);

  // 生成正弦波路径
  auto [path_x, path_y, cyaw, curvature, direction] =
      PathGenerator::generateSineWavePath(amplitude, frequency, length, step);

  std::cout << "Path generated." << std::endl;

  // 起始点
  VehicleState node(path_x[0], path_y[0], cyaw[0], 0.1, Gear::DRIVE);

  // 仿真记录
  double time = 0.0;
  std::vector<double> x = {node.x}, y = {node.y}, yaw = {node.yaw},
                      v = {node.v};
  std::vector<double> t = {0.0}, d = {0.0}, a = {0.0};
  std::vector<double> delta_opt, a_opt;
  double a_exc_old = 0.0, delta_exc_old = 0.0;

  ros::Subscriber sub = nh.subscribe("/odom", 1000, odomCallback);

  // 成员变量定义发布者
  ros::Publisher pub_vel_left_rear_wheel;
  ros::Publisher pub_vel_right_rear_wheel;
  ros::Publisher pub_vel_left_front_wheel;
  ros::Publisher pub_vel_right_front_wheel;

  ros::Publisher pub_pos_left_steering_hinge;
  ros::Publisher pub_pos_right_steering_hinge;
  // 创建发布者对象，用于发送控制命令
  pub_vel_left_rear_wheel = nh.advertise<std_msgs::Float64>(
      "/racebot/left_rear_wheel_velocity_controller/command", 1);
  pub_vel_right_rear_wheel = nh.advertise<std_msgs::Float64>(
      "/racebot/right_rear_wheel_velocity_controller/command", 1);
  pub_vel_left_front_wheel = nh.advertise<std_msgs::Float64>(
      "/racebot/left_front_wheel_velocity_controller/command", 1);
  pub_vel_right_front_wheel = nh.advertise<std_msgs::Float64>(
      "/racebot/right_front_wheel_velocity_controller/command", 1);
  pub_pos_left_steering_hinge = nh.advertise<std_msgs::Float64>(
      "/racebot/left_steering_hinge_position_controller/command", 1);
  pub_pos_right_steering_hinge = nh.advertise<std_msgs::Float64>(
      "/racebot/right_steering_hinge_position_controller/command", 1);

  double wheelbase = l_f + l_r;  // 计算车辆的轴距

  LatController lat_controller;  // 创建横向控制器实例
  LonController lon_controller;  // 创建纵向控制器实例

  // 初始化轨迹分析器
  TrajectoryAnalyzer trajectory_analyzer(path_x, path_y, cyaw, curvature);

  std::cout << trajectory_analyzer.x_.size() << std::endl;  // 打印轨迹点的数量

  // 对路径上的每一个点进行模拟
  // for (size_t i = 0; i < path_x.size() - 1; ++i) {
  double target_speed = 25.0 / 3.6;  // 假设恒定的目标速度
  // publish_trajectory(path_x, path_y, marker_pub);
  ros::Rate loop_rate(20);  // 50 Hz
  while (1) {
    publish_trajectory(path_x, path_y, marker_pub);

    // x.push_back(node.x);
    // y.push_back(node.y);
    // yaw.push_back(node.yaw);
    // v.push_back(node.v);
    // t.push_back(time);

    double dist =
        std::hypot(node.x - path_x[path_x.size() - 1],
                   node.y - path_y[path_y.size() - 1]);  // 计算到下一个点的距离
    std::cout << "Dist: " << dist << std::endl;  // 打印距离信息
    if (dist <= 0.1) {
      std_msgs::Float64 displacement_cmd;
      std_msgs::Float64 a_acker_cmd;
      displacement_cmd.data = 0;
      a_acker_cmd.data = 0;
      pub_vel_left_rear_wheel.publish(displacement_cmd);
      pub_vel_right_rear_wheel.publish(displacement_cmd);
      pub_vel_left_front_wheel.publish(displacement_cmd);
      pub_vel_right_front_wheel.publish(displacement_cmd);
      pub_pos_left_steering_hinge.publish(a_acker_cmd);
      pub_pos_right_steering_hinge.publish(a_acker_cmd);
      break;  // 如果到达终点，停止模拟
    }

    // 使用轨迹分析器计算控制命令
    auto [delta_opt, theta_e, e_cg] =
        lat_controller.computeControlCommand(node, trajectory_analyzer);

    double a_opt = lon_controller.computeControlCommand(
        target_speed, node, dist);  // 计算纵向控制命令

    node.x = x_gazebo;
    node.y = y_gazebo;
    node.yaw = yaw_gazebo;
    node.v = v_gazebo;
    node.e_cg = e_cg;
    node.theta_e = theta_e;

    // 转换为阿克曼命令
    double delta_cmd = delta_opt;
    double a_cmd = a_opt;
    time += ts;
    std::cout << "Time: " << time << std::endl;  // 打印时间信息

    // 发布gazebo控制命令
    // 根据接收到的阿克曼驱动命令设置油门和转向角
    double displacement = node.v * ts + 0.5 * a_cmd * ts * ts;

    std_msgs::Float64 displacement_cmd;
    std_msgs::Float64 a_acker_cmd;
    displacement_cmd.data = displacement * 100;
    a_acker_cmd.data = delta_cmd;
    // 打印displacement和delta_cmd

    // 发布计算得到的油门和转向值到相应的控制器
    pub_vel_left_rear_wheel.publish(displacement_cmd);
    pub_vel_right_rear_wheel.publish(displacement_cmd);
    pub_vel_left_front_wheel.publish(displacement_cmd);
    pub_vel_right_front_wheel.publish(displacement_cmd);
    pub_pos_left_steering_hinge.publish(a_acker_cmd);
    pub_pos_right_steering_hinge.publish(a_acker_cmd);

    // 等待循环的下一个周期
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
