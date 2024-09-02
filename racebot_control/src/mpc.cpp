#include "mpc.h"
#include "OsqpEigen/OsqpEigen.h"
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cmath> // 用于计算数学函数
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
template <typename T>
bool getParam(ros::NodeHandle &nh, const std::string &name, T &var) {
  if (nh.getParam(name, var)) {
    return true;
  } else {
    ROS_WARN("Failed to get %s parameter, setting default", name.c_str());
    return false;
  }
}

bool getMatrixFromParam(const ros::NodeHandle &nh,
                        const std::string &param_name,
                        std::vector<std::vector<double>> &matrix, int rows,
                        int cols) {
  std::vector<double> flat_matrix;
  if (!nh.getParam(param_name, flat_matrix) ||
      flat_matrix.size() != rows * cols) {
    ROS_ERROR("Failed to get parameter %s or size mismatch",
              param_name.c_str());
    return false;
  }

  matrix.resize(rows, std::vector<double>(cols));
  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      matrix[i][j] = flat_matrix[i * cols + j];
    }
  }
  return true;
}

MPCParams MPCParams_;
vehicleParams vehicleParams_;

void loadVehicleConfig(ros::NodeHandle &nh) {
  getParam(nh, "vehicle_config/RF", vehicleParams_.RF);
  getParam(nh, "vehicle_config/RB", vehicleParams_.RB);
  getParam(nh, "vehicle_config/W", vehicleParams_.W);
  getParam(nh, "vehicle_config/WD", vehicleParams_.WD);
  getParam(nh, "vehicle_config/WB", vehicleParams_.WB);
  getParam(nh, "vehicle_config/TR", vehicleParams_.TR);
  getParam(nh, "vehicle_config/TW", vehicleParams_.TW);
  getParam(nh, "vehicle_config/steer_max", vehicleParams_.steer_max);
  getParam(nh, "vehicle_config/steer_change_max",
           vehicleParams_.steer_change_max);
  getParam(nh, "vehicle_config/speed_max", vehicleParams_.speed_max);
  getParam(nh, "vehicle_config/speed_min", vehicleParams_.speed_min);
  getParam(nh, "vehicle_config/acceleration_max",
           vehicleParams_.acceleration_max);

  ROS_INFO("Vehicle Configuration Loaded.");
}

void loadMPCConfig(ros::NodeHandle &nh) {
  getParam(nh, "mpc_config/NX", MPCParams_.NX);
  getParam(nh, "mpc_config/NU", MPCParams_.NU);
  getParam(nh, "mpc_config/T", MPCParams_.T);
  getMatrixFromParam(nh, "mpc_config/Q", MPCParams_.Q, 4, 4);
  getMatrixFromParam(nh, "mpc_config/Qf", MPCParams_.Qf, 4, 4);
  getMatrixFromParam(nh, "mpc_config/R", MPCParams_.R, 2, 2);
  getMatrixFromParam(nh, "mpc_config/Rd", MPCParams_.Rd, 2, 2);
  getParam(nh, "mpc_config/dist_stop", MPCParams_.dist_stop);
  getParam(nh, "mpc_config/speed_stop", MPCParams_.speed_stop);
  getParam(nh, "mpc_config/time_max", MPCParams_.time_max);
  getParam(nh, "mpc_config/iter_max", MPCParams_.N_IND);
  getParam(nh, "mpc_config/target_speed", MPCParams_.target_speed);
  getParam(nh, "mpc_config/dt", MPCParams_.dt);
  getParam(nh, "mpc_config/d_dist", MPCParams_.d_dist);
  getParam(nh, "mpc_config/du_res", MPCParams_.du_res);

  ROS_INFO("MPC Configuration Loaded.");
}

class Spline {
private:
  std::vector<double> a, b, c, d,
      w; // a, b, c, d 是样条曲线的系数，w 未在代码中使用
  const std::vector<double> &x; // x 值的常量引用
  const std::vector<double> &y; // y 值的常量引用
  size_t nx;                    // x 值的数量

  // 计算相邻点之间的差值
  std::vector<double> calcDiff(const std::vector<double> &v) {
    std::vector<double> result(v.size() - 1);
    for (size_t i = 0; i < v.size() - 1; ++i) {
      result[i] = v[i + 1] - v[i];
    }
    return result;
  }

  // 计算系数矩阵A，用于求解c
  std::vector<std::vector<double>> calcA(const std::vector<double> &h) {
    std::vector<std::vector<double>> A(nx, std::vector<double>(nx, 0.0));
    A[0][0] = 1.0;
    for (size_t i = 1; i < nx - 1; ++i) {
      A[i][i] = 2.0 * (h[i - 1] + h[i]);
      A[i][i - 1] = h[i - 1];
      A[i - 1][i] = h[i - 1];
    }
    A[nx - 1][nx - 1] = 1.0;
    return A;
  }

  // 计算向量B，用于求解c
  std::vector<double> calcB(const std::vector<double> &h) {
    std::vector<double> B(nx, 0.0);
    for (size_t i = 1; i < nx - 1; ++i) {
      B[i] = 3.0 * ((y[i + 1] - y[i]) / h[i] - (y[i] - y[i - 1]) / h[i - 1]);
    }
    return B;
  }

  // 解线性方程组，更新c
  void solve(std::vector<std::vector<double>> &A, std::vector<double> &B) {
    Eigen::MatrixXd matA(A.size(), A.size());
    Eigen::VectorXd vecB(B.size());

    for (size_t i = 0; i < A.size(); i++) {
      for (size_t j = 0; j < A.size(); j++) {
        matA(i, j) = A[i][j];
      }
      vecB(i) = B[i];
    }

    Eigen::VectorXd vecC = matA.colPivHouseholderQr().solve(vecB);
    for (size_t i = 0; i < c.size(); i++) {
      c[i] = vecC(i);
    }
  }

  // 根据给定的x值找到其在数组x中的位置索引
  size_t searchIndex(double xi) {
    auto it = std::lower_bound(x.begin(), x.end(), xi);
    return std::distance(x.begin(), it) - 1;
  }

public:
  // 构造函数，初始化并计算样条曲线参数
  Spline(const std::vector<double> &x, const std::vector<double> &y)
      : x(x), y(y) {
    nx = x.size();
    std::vector<double> h = calcDiff(x); // 计算x间隔

    a = y; // 直接使用输入的y值作为a系数
    std::vector<std::vector<double>> A = calcA(h);
    std::vector<double> B = calcB(h);
    c.resize(nx);
    solve(A, B); // 求解c系数

    b.resize(nx - 1);
    d.resize(nx - 1);
    for (size_t i = 0; i < nx - 1; ++i) {
      d[i] = (c[i + 1] - c[i]) / (3.0 * h[i]);
      b[i] = (a[i + 1] - a[i]) / h[i] - h[i] * (c[i + 1] + 2.0 * c[i]) / 3.0;
    }
  }

  // 计算给定t时样条曲线的y值
  double calc(double t) {
    if (t < x.front() || t > x.back()) {
      return NAN; // t值超出范围，返回NaN
    }
    size_t i = searchIndex(t);
    double dx = t - x[i];
    return y[i] + b[i] * dx + c[i] * std::pow(dx, 2) + d[i] * std::pow(dx, 3);
  }

  // 计算给定t时样条曲线的一阶导数值
  double calcd(double t) {
    if (t < x.front() || t > x.back()) {
      return NAN; // t值超出范围，返回NaN
    }
    size_t i = searchIndex(t);
    double dx = t - x[i];
    return b[i] + 2.0 * c[i] * dx + 3.0 * d[i] * std::pow(dx, 2);
  }

  // 计算给定t时样条曲线的二阶导数值
  double calcdd(double t) {
    if (t < x.front() || t > x.back()) {
      return NAN; // t值超出范围，返回NaN
    }
    size_t i = searchIndex(t);
    double dx = t - x[i];
    return 2.0 * c[i] + 6.0 * d[i] * dx;
  }
};

class Spline2D {
public:
  std::vector<double> s; // 存储弧长参数
  Spline sx, sy;         // x和y坐标的样条曲线对象

  // 计算沿曲线的累积弧长
  std::vector<double> calcS(const std::vector<double> &x,
                            const std::vector<double> &y) {
    std::vector<double> dx(x.size() - 1); // x的差分
    std::vector<double> dy(y.size() - 1); // y的差分
    // 计算x和y的差值
    std::transform(x.begin() + 1, x.end(), x.begin(), dx.begin(),
                   std::minus<double>());
    std::transform(y.begin() + 1, y.end(), y.begin(), dy.begin(),
                   std::minus<double>());
    std::vector<double> ds(dx.size()); // 存储弧长增量
    // 计算每段的弧长
    for (size_t i = 0; i < dx.size(); ++i) {
      ds[i] = std::sqrt(dx[i] * dx[i] + dy[i] * dy[i]);
    }
    std::vector<double> s; // 弧长数组
    s.push_back(0);        // 初始化起点弧长为0
    // 计算累积弧长
    std::partial_sum(ds.begin(), ds.end(), std::back_inserter(s));
    return s;
  }

  // 构造函数，初始化弧长和x、y的样条曲线
  Spline2D(const std::vector<double> &x, const std::vector<double> &y)
      : s(calcS(x, y)), sx(s, x), sy(s, y) {}

  // 根据弧长值计算曲线上的位置（x，y）
  std::pair<double, double> calcPosition(double s_val) {
    double x = sx.calc(s_val); // 计算x坐标
    double y = sy.calc(s_val); // 计算y坐标
    return {x, y};             // 返回坐标对
  }

  // 根据弧长值计算曲线的曲率
  double calcCurvature(double s_val) {
    double dx = sx.calcd(s_val);                           // x的一阶导数
    double ddx = sx.calcdd(s_val);                         // x的二阶导数
    double dy = sy.calcd(s_val);                           // y的一阶导数
    double ddy = sy.calcdd(s_val);                         // y的二阶导数
    double denominator = std::pow(dx * dx + dy * dy, 1.5); // 计算分母
    // 如果分母为0，则返回NaN，否则计算曲率
    return denominator == 0 ? std::numeric_limits<double>::quiet_NaN()
                            : (ddy * dx - ddx * dy) / denominator;
  }

  // 根据弧长值计算曲线的偏航角（朝向角度）
  double calcYaw(double s_val) {
    double dx = sx.calcd(s_val); // x的一阶导数
    double dy = sy.calcd(s_val); // y的一阶导数
    return std::atan2(dy, dx);   // 计算偏航角
  }
};

std::vector<double> linspace(double start, double end, double step) {
  std::vector<double> result; // 创建一个空的 double 类型向量用于存储结果

  // 使用 for 循环从 start 开始，每次增加 step，直到 end
  for (double val = start; val <= end; val += step) {
    result.push_back(val); // 将当前的值添加到结果向量中
  }

  return result; // 返回生成的等差数列
}

void calc_spline_course(const std::vector<double> &x,
                        const std::vector<double> &y, double ds,
                        std::vector<double> &rx, std::vector<double> &ry,
                        std::vector<double> &ryaw, std::vector<double> &rk,
                        std::vector<double> &s) {
  std::cout << "calc_spline_course" << std::endl;
  Spline2D sp(x, y); // 创建二维样条曲线对象
  std::cout << "s.back(): " << sp.s.back() << std::endl;
  s = linspace(0, sp.s.back(), ds); //生成从0到样条曲线总长度，间隔为ds的数组

  for (double i_s : s) {                  // 遍历每一个曲线长度值
    auto [ix, iy] = sp.calcPosition(i_s); // 计算当前位置的x, y坐标
    rx.push_back(ix); // 将计算得到的x坐标添加到列表中
    ry.push_back(iy); // 将计算得到的y坐标添加到列表中
    ryaw.push_back(sp.calcYaw(i_s)); // 计算并添加当前位置的偏航角
    rk.push_back(sp.calcCurvature(i_s)); // 计算并添加当前位置的曲率
  }
}

double pi_2_pi(double angle) {
  while (angle > M_PI)
    angle -= 2.0 * M_PI;
  while (angle < -M_PI)
    angle += 2.0 * M_PI;
  return angle;
}

std::vector<double> calc_speed_profile(const std::vector<double> &cx,
                                       const std::vector<double> &cy,
                                       const std::vector<double> &cyaw,
                                       double target_speed) {
  std::vector<double> speed_profile(
      cx.size(), target_speed); // 初始化速度剖面，默认值为目标速度
  double direction = 1.0;       // 前进方向

  // 设置停止点
  for (size_t i = 0; i < cx.size() - 1; ++i) {
    double dx = cx[i + 1] - cx[i]; // 计算相邻两个点的x方向差值
    double dy = cy[i + 1] - cy[i]; // 计算相邻两个点的y方向差值

    double move_direction = std::atan2(dy, dx); // 计算运动方向的角度

    if (dx != 0.0 && dy != 0.0) { // 如果dx和dy都不为零
      double dangle = std::abs(pi_2_pi(
          move_direction - cyaw[i])); // 计算运动方向与路径偏航角的角度差
      if (dangle >= M_PI / 4.0) { // 如果角度差大于等于45度
        direction = -1.0;         // 方向设为后退
      } else {
        direction = 1.0; // 否则设为前进
      }
    }

    if (direction != 1.0) {             // 如果方向不是前进
      speed_profile[i] = -target_speed; // 设置当前点的速度为负目标速度
    } else {
      speed_profile[i] = target_speed; // 否则设置为正目标速度
    }
  }

  speed_profile.back() = 0.0; // 最后一个点的速度设为0

  return speed_profile; // 返回速度剖面
}

class Node {
public:
  double x, y, yaw, v; // x, y坐标，yaw为车辆朝向角度，v为速度
  double direct;       // 表示运动方向的因子，通常为1或-1

  // 构造函数，带默认参数
  Node(double x = 0.0, double y = 0.0, double yaw = 0.0, double v = 0.0,
       double direct = 1.0)
      : x(x), y(y), yaw(yaw), v(v), direct(direct) {
  } // 使用初始化列表来初始化变量

  // 更新车辆状态的函数
  void update(double a, double delta, double direct) {
    delta = limit_input_delta(delta); // 限制转向角度到允许的范围内
    x += v * cos(yaw) * MPCParams_.dt; // 更新x坐标
    y += v * sin(yaw) * MPCParams_.dt; // 更新y坐标
    yaw +=
        v / vehicleParams_.WB * tan(delta) * MPCParams_.dt; // 更新车辆朝向角度
    this->direct = direct;                 // 更新运动方向
    v += this->direct * a * MPCParams_.dt; // 根据加速度更新速度
    v = limit_speed(v);                    // 限制速度到允许的范围内
  }

  // 限制输入的转向角度函数
  double limit_input_delta(double delta) {
    if (delta >= vehicleParams_.steer_max) {
      return vehicleParams_.steer_max; // 如果转向角度超过最大值，返回最大值
    }
    if (delta <= -vehicleParams_.steer_max) {
      return -vehicleParams_.steer_max; // 如果转向角度低于最小值，返回最小值
    }
    return delta; // 如果在允许范围内，返回原值
  }

  // 限制速度的函数
  double limit_speed(double v) {
    if (v >= vehicleParams_.speed_max) {
      return vehicleParams_.speed_max; // 如果速度超过最大速度，返回最大速度
    }
    if (v <= vehicleParams_.speed_min) {
      return vehicleParams_.speed_min; // 如果速度低于最小速度，返回最小速度
    }
    return v; // 如果在允许范围内，返回原速度
  }
};

class PATH {
public:
  std::vector<double> cx, cy, cyaw,
      ck; // 路径上各点的x坐标，y坐标，朝向角度和曲率
  int length, ind_old; // 路径长度和上一次查询的索引

  // 构造函数，初始化路径
  PATH(const std::vector<double> &cx, const std::vector<double> &cy,
       const std::vector<double> &cyaw, const std::vector<double> &ck)
      : cx(cx), cy(cy), cyaw(cyaw), ck(ck), length(cx.size()), ind_old(0) {
  } // 使用初始化列表初始化成员变量

  // 找到给定节点最近的路径点的索引和横向误差
  std::pair<int, double> nearest_index(const Node &node) {
    int search_up_to =
        std::min(ind_old + MPCParams_.N_IND, length); // 计算搜索范围的上限
    std::vector<double> dx, dy, dist; // 用于存储差值和距离

    for (int i = ind_old; i < search_up_to; ++i) {
      dx.push_back(node.x - cx[i]); // 计算x坐标差
      dy.push_back(node.y - cy[i]); // 计算y坐标差
      dist.push_back(
          std::hypot(dx.back(), dy.back())); // 计算欧几里得距离并存储
    }

    auto min_iter =
        std::min_element(dist.begin(), dist.end()); // 找到最小距离元素的迭代器
    int ind_in_N =
        std::distance(dist.begin(), min_iter); // 计算最小距离点的索引
    int ind = ind_old + ind_in_N;              // 计算全局索引
    ind_old = ind;                             // 更新搜索的起始索引

    // 计算后轴向量旋转90度的结果
    Eigen::Vector2d rear_axle_vec_rot_90(std::cos(node.yaw + M_PI / 2.0),
                                         std::sin(node.yaw + M_PI / 2.0));

    // 计算目标点到后轴的向量
    Eigen::Vector2d vec_target_2_rear(dx[ind_in_N], dy[ind_in_N]);

    // 计算横向误差
    double er = rear_axle_vec_rot_90.dot(vec_target_2_rear);

    return {ind, er}; // 返回最近点的索引和横向误差
  }
};

double x_ = 0.0, y_ = 0.0, yaw_ = 0.0, v_ = 0.0;

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
  ROS_INFO_STREAM("Position -> x: " << std::fixed << std::setprecision(2) << x
                                    << ", y: " << y);
  ROS_INFO_STREAM("Speed: " << std::fixed << std::setprecision(2) << speed);
  ROS_INFO_STREAM("Yaw: " << std::fixed << std::setprecision(2) << yaw);
  x_ = x;
  y_ = y;
  yaw_ = yaw;
  v_ = speed;
}

std::vector<std::vector<double>>
calc_ref_trajectory_in_T_step(const Node &node, PATH &ref_path,
                              const std::vector<double> &sp, int &target_ind) {
  std::vector<std::vector<double>> z_ref(
      MPCParams_.NX,
      std::vector<double>(MPCParams_.T + 1,
                          0.0)); // Initialize reference trajectory array

  std::pair<int, double> nearest_index =
      ref_path.nearest_index(node); // Get the nearest index on the path
                                    // to the current node
  target_ind = nearest_index.first; // Set the target index
  // std::cout << "target_ind: " << target_ind << std::endl;
  // Initialize reference trajectory
  z_ref[0][0] = ref_path.cx[target_ind];   // x-coordinate
  z_ref[1][0] = ref_path.cy[target_ind];   // y-coordinate
  z_ref[2][0] = sp[target_ind];            // Speed
  z_ref[3][0] = ref_path.cyaw[target_ind]; // Yaw angle

  double dist_move = 0.0; // Initialize moved distance
  // std::cout << "node.v: " << node.v << std::endl;
  for (int i = 1; i <= MPCParams_.T; ++i) {
    dist_move +=
        std::abs(node.v) * MPCParams_.dt; // Calculate the moved distance
    // std::cout << "dist_move: " << dist_move << std::endl;
    int ind_move = static_cast<int>(
        std::round(dist_move)); // Calculate the index increment
    // std::cout << "ind_move: " << ind_move << std::endl;
    int index = std::min(target_ind + ind_move,
                         ref_path.length -
                             1); // Ensure the index does not exceed path length

    // std::cout << "index: " << index << std::endl;

    // Update reference trajectory
    z_ref[0][i] = ref_path.cx[index];   // x-coordinate
    z_ref[1][i] = ref_path.cy[index];   // y-coordinate
    z_ref[2][i] = sp[index];            // Speed
    z_ref[3][i] = ref_path.cyaw[index]; // Yaw angle
  }

  return z_ref; // Return the reference trajectory
}

// Returns A, B matrices for linear discrete-time dynamic model
void calc_linear_discrete_model(double v, double phi, double delta,
                                Eigen::MatrixXd &A, Eigen::MatrixXd &B) {
  // Initialize matrices A and B, and vector C
  
  A = Eigen::MatrixXd(4, 4);
  B = Eigen::MatrixXd(4, 2);

  // Compute elements of A
  A << 1.0, 0.0, MPCParams_.dt * std::cos(phi),
      -MPCParams_.dt * v * std::sin(phi), 0.0, 1.0,
      MPCParams_.dt * std::sin(phi), MPCParams_.dt * v * std::cos(phi), 0.0,
      0.0, 1.0, 0.0, 0.0, 0.0,
      MPCParams_.dt * std::tan(delta) / vehicleParams_.WB, 1.0;

  // Compute elements of B
  B << 0.0, 0.0, 0.0, 0.0, MPCParams_.dt, 0.0, 0.0,
      MPCParams_.dt * v / (vehicleParams_.WB * std::pow(std::cos(delta), 2));
}

Eigen::MatrixXd convertToEigen(const std::vector<std::vector<double>> &input) {
  if (input.empty() || input[0].empty()) {
    return Eigen::MatrixXd(); // 返回一个空的Eigen矩阵
  }

  int rows = input.size();
  int cols = input[0].size();

  // 初始化Eigen矩阵
  Eigen::MatrixXd matrix(rows, cols);

  // 填充矩阵
  for (int i = 0; i < rows; ++i) {
    if (input[i].size() != cols) {
      throw std::runtime_error("All inner vectors must be of the same size");
    }
    for (int j = 0; j < cols; ++j) {
      matrix(i, j) = input[i][j];
    }
  }

  return matrix;
}

void linear_mpc_control(const std::vector<std::vector<double>> &x_ref_T,
                        const std::vector<double> &z0,
                        std::vector<double> &a_opt,
                        std::vector<double> &delta_opt, double &a_exc_old,
                        double &delta_exc_old) {
  // std::cout << "linear_mpc_control" << std::endl;
  // std::cout << "a_exc: " << a_exc_old << ", delta_exc: " << delta_exc_old
  //           << std::endl;
  // Initialize matrices A, B
  Eigen::MatrixXd A, B;
  //打印z0[2], z0[3], delta_exc_old
  // std::cout << "z0[2]: " << z0[2] << ", z0[3]: " << z0[3]
  //           << ", delta_exc_old: " << delta_exc_old << std::endl;
  calc_linear_discrete_model(z0[2], z0[3], delta_exc_old, A, B);
  // 打印A和B
  // std::cout << "A: " << std::endl << A << std::endl;
  // std::cout << "B: " << std::endl << B << std::endl;

  // Initialize matrices Q, R, Rd
  Eigen::MatrixXd Q = convertToEigen(MPCParams_.Q);
  Eigen::MatrixXd R = convertToEigen(MPCParams_.R);
  Eigen::MatrixXd Qf = convertToEigen(MPCParams_.Qf);
  //打印Q, R, Qf
  // std::cout << "Q: " << std::endl << Q << std::endl;
  // std::cout << "R: " << std::endl << R << std::endl;
  // std::cout << "Qf: " << std::endl << Qf << std::endl;

  // Initialize vector x0
  Eigen::VectorXd x0(4);
  x0 << z0[0], z0[1], z0[2], z0[3];
  //打印x0
  // std::cout << "x0: " << std::endl << x0 << std::endl;

  // Initialize vector x_ref_T
  Eigen::MatrixXd x_ref_T_mat(4, MPCParams_.T + 1);

  // 打印x_ref_T
  for (size_t i = 0; i < x_ref_T.size(); ++i) {
    for (size_t j = 0; j < x_ref_T[i].size(); ++j) {
      // std::cout << x_ref_T[i][j] << " ";
      x_ref_T_mat(i, j) = x_ref_T[i][j];
    }
    // std::cout << std::endl;
  }
  Eigen::VectorXd x_ref_T_vec = Eigen::VectorXd::Zero((MPCParams_.T + 1) * 4);
  // std::cout << "x_ref_T_vec size: " << x_ref_T_vec.size() << std::endl;
  for (int i = 0; i < MPCParams_.T + 1; ++i) {
    x_ref_T_vec.segment(i * 4, 4) = x_ref_T_mat.col(i);
  }
  // std::cout << "x_ref_T_vec: " << std::endl << x_ref_T_vec << std::endl;

  // Xt = [x0, x1, x2, ..., xT]
  Eigen::MatrixXd Xt(4, MPCParams_.T + 1);
  // Ut = [u0, u1, u2, ..., uT-1]
  Eigen::MatrixXd Ut(2, MPCParams_.T);

  // x(k+1) = A * x(k) + B * u(k)
  // Xt = M * xt + C * Ut
  // M = [I A A^2 ... A^T]T
  Eigen::MatrixXd M;
  // 填充M
  int state_dim = A.rows(); // Dimension of the state (4 in this case)
  int M_rows = state_dim * (MPCParams_.T + 1);
  int M_cols = state_dim;

  // Resize M just in case it's not the right size
  M.resize(M_rows, M_cols);
  M.setZero();

  // Start with I (the identity matrix) for A^0
  Eigen::MatrixXd A_power = Eigen::MatrixXd::Identity(state_dim, state_dim);

  // Populate M with powers of A
  for (int i = 0; i <= MPCParams_.T; ++i) {
    // Set the block starting at (i*state_dim, 0)
    M.block(i * state_dim, 0, state_dim, state_dim) = A_power;
    // Compute the next power of A for the next iteration
    A_power = A_power * A;
  }
  // std::cout << "Matrix M:\n" << M << std::endl;
  // std::cout << "Matrix M size: " << M.rows() << " x " << M.cols() <<
  // std::endl; C = [0 0 0 ... 0; B 0 0 ... 0; AB B 0 ... 0; A^2B AB B ... 0;
  // ...; A^T-1B
  // ... 0 B]
  Eigen::MatrixXd C;
  // 填充C
  int input_dim = B.cols(); // Dimension of the input (2 in this case)
  int C_rows = state_dim * (MPCParams_.T + 1);
  int C_cols = MPCParams_.T * input_dim;
  C.resize(C_rows, C_cols);
  C.setZero();
  // std::cout << "Matrix C size: " << C.rows() << " x " << C.cols() <<
  // std::endl;

  Eigen::MatrixXd A_power_B = B; // Start with B, which is A^0 * B

  for (int i = MPCParams_.T; i >= 1;
       i--) { // Note the loop starts from 1 because the first row block is
    // zeros
    // std::cout << "i: " << i << std::endl;
    for (int j = 0; j < i; ++j) {
      // Place the matrix A^(i-j-1)*B at position (i*state_dim, j*input_dim)
      // Note the (i-j-1) as we start from A^0 = B at j = i-1 and decrease
      C.block((j + 1 + (MPCParams_.T - i)) * state_dim, (j)*input_dim,
              state_dim, input_dim) = A_power_B;
      // std::cout << "Matrix C:\n" << C << std::endl;
    }
    // Update A_power_B for the next row
    A_power_B = A * A_power_B;
    // std::cout << "Matrix A_power_B:\n" << A_power_B << std::endl;
  }

  // std::cout << "Matrix C:\n" << C << std::endl;
  // std::cout << "Matrix C size: " << C.rows() << " x " << C.cols() <<
  // std::endl;

  // Initialize matrix Q_bar
  Eigen::MatrixXd Q_bar = Eigen::MatrixXd::Zero((MPCParams_.T + 1) * Q.rows(),
                                                (MPCParams_.T + 1) * Q.cols());
  // std::cout << "Matrix Q_bar size: " << Q_bar.rows() << " x " << Q_bar.cols()
  //           << std::endl;
  for (int i = 0; i <= MPCParams_.T; ++i) {
    if (i == MPCParams_.T) {
      Q_bar.block(i * Q.rows(), i * Q.cols(), Q.rows(), Q.cols()) = Qf;
    }
    Q_bar.block(i * Q.rows(), i * Q.cols(), Q.rows(), Q.cols()) = Q;
  }
  // std::cout << "Matrix Q_bar:\n" << Q_bar << std::endl;

  // Initialize matrix R_bar
  Eigen::MatrixXd R_bar =
      Eigen::MatrixXd::Zero(MPCParams_.T * R.rows(), MPCParams_.T * R.cols());
  // std::cout << "Matrix R_bar size: " << R_bar.rows() << " x " << R_bar.cols()
  //           << std::endl;
  for (int i = 0; i < MPCParams_.T; ++i) {
    R_bar.block(i * R.rows(), i * R.cols(), R.rows(), R.cols()) = R;
  }
  // std::cout << "Matrix R_bar:\n" << R_bar << std::endl;

  // Initialize matrix hessian
  Eigen::MatrixXd hessian = C.transpose() * Q_bar * C + R_bar;
  // std::cout << "Matrix hessian size: " << hessian.rows() << " x "
  //           << hessian.cols() << std::endl;
  // std::cout << "Matrix hessian:\n" << hessian << std::endl;

  // Initialize matrix gradient
  Eigen::RowVectorXd gradient = (M * x0 - x_ref_T_vec).transpose() * Q_bar * C;
  // std::cout << "Matrix gradient size: " << gradient.rows() << " x "
  // << gradient.cols() << std::endl;
  // std::cout << "Matrix gradient:\n" << gradient << std::endl;

  // Initialize matrix A_ieq
  Eigen::MatrixXd A_ieq =
      Eigen::MatrixXd::Zero(input_dim * MPCParams_.T, input_dim * MPCParams_.T);
  // std::cout << "Matrix A_ieq size: " << A_ieq.rows() << " x " << A_ieq.cols()
  //           << std::endl;

  // 填充A_ieq，对角线上的元素为1
  for (int i = 0; i < A_ieq.rows(); ++i) {
    A_ieq(i, i) = 1;
  }
  // std::cout << "Matrix A_ieq:\n" << A_ieq << std::endl;

  // Initialize vector lower_bound
  Eigen::VectorXd lower_bound =
      Eigen::VectorXd::Zero(A_ieq.rows()); // Initialize lower bound
  // std::cout << "Vector lower_bound size: " << lower_bound.rows() <<
  // std::endl; 填充lower_bound
  for (int i = 0; i < lower_bound.rows(); ++i) {
    if (i % 2 == 0) {
      lower_bound(i) = -vehicleParams_.acceleration_max;
    } else {
      lower_bound(i) = -vehicleParams_.steer_max;
    }
  }
  // std::cout << "Vector lower_bound:\n" << lower_bound << std::endl;
  // Initialize vector upper_bound
  Eigen::VectorXd upper_bound =
      Eigen::VectorXd::Zero(A_ieq.rows()); // Initialize upper bound
  // std::cout << "Vector upper_bound size: " << upper_bound.rows() <<
  // std::endl; 填充upper_bound
  for (int i = 0; i < upper_bound.rows(); ++i) {
    if (i % 2 == 0) {
      upper_bound(i) = vehicleParams_.acceleration_max;
    } else {
      upper_bound(i) = vehicleParams_.steer_max;
    }
  }
  // std::cout << "Vector upper_bound:\n" << upper_bound << std::endl;
  //打印标准QP问题
  // std::cout << "Standard QP problem:" << std::endl;
  // std::cout << "Hessian:\n" << hessian << std::endl;
  // std::cout << "Gradient:\n" << gradient << std::endl;
  // std::cout << "A_ieq:\n" << A_ieq << std::endl;
  // std::cout << "Lower bound:\n" << lower_bound << std::endl;
  // std::cout << "Upper bound:\n" << upper_bound << std::endl;
  // std::cout << "Hessian.size(): " << hessian.rows() << " x " <<
  // hessian.cols()
  //           << std::endl;
  // std::cout << "Gradient.size(): " << gradient.rows() << " x "
  //           << gradient.cols() << std::endl;
  // std::cout << "A_ieq.size(): " << A_ieq.rows() << " x " << A_ieq.cols()
  //           << std::endl;
  // std::cout << "Lower bound.size(): " << lower_bound.rows() << std::endl;
  // std::cout << "Upper bound.size(): " << upper_bound.rows() << std::endl;

  Eigen::SparseMatrix<double> sparse_hessian = hessian.sparseView();
  Eigen::SparseMatrix<double> sparse_A_ieq = A_ieq.sparseView();
  // Solve the QP problem using OSQP
  // Create a new solver
  // 设置OSQP求解器
  OsqpEigen::Solver solver;
  solver.settings()->setVerbosity(true);

  // 设置QP问题
  solver.data()->setNumberOfVariables(hessian.rows());
  solver.data()->setNumberOfConstraints(hessian.rows());
  if (!solver.data()->setHessianMatrix(sparse_hessian))
    std::cout << "Failed to set Hessian matrix" << std::endl;
  if (!solver.data()->setGradient(gradient))
    std::cout << "Failed to set gradient" << std::endl;
  if (!solver.data()->setLinearConstraintsMatrix(sparse_A_ieq))
    std::cout << "Failed to set linear constraints matrix" << std::endl;
  if (!solver.data()->setLowerBound(lower_bound))
    std::cout << "Failed to set lower bound" << std::endl;
  if (!solver.data()->setUpperBound(upper_bound))
    std::cout << "Failed to set upper bound" << std::endl;

  if (!solver.initSolver())
    std::cout << "Failed to initialize solver" << std::endl;

  // 求解QP问题
  solver.solveProblem(); // Updated to use the new API
  Eigen::VectorXd solution = solver.getSolution();
  // std::cout << "Solution:" << solution << std::endl;
  a_opt.push_back(solution(0));
  delta_opt.push_back(solution(1));
}

void publish_trajectory(const std::vector<double> &cx,
                        const std::vector<double> &cy,
                        ros::Publisher &marker_pub) {
  visualization_msgs::Marker points; // 创建一个Marker消息，用于显示点
  points.header.frame_id = "odom";   // 设置frame_id为"odom"
  points.header.stamp = ros::Time::now(); // 设置时间戳为当前时间
  points.ns = "trajectory"; // 设置这组Marker的命名空间为"trajectory"
  points.action = visualization_msgs::Marker::ADD; // 设置动作为添加（ADD）
  points.pose.orientation.w = 1.0; // 设置方向的四元数，这里表示无旋转
  points.id = 0; // 设置这个Marker的ID，可以用于识别和修改
  points.type = visualization_msgs::Marker::POINTS; // 设置Marker类型为点
  points.scale.x = 0.01;                            // 设置点的X方向大小
  points.scale.y = 0.01;                            // 设置点的Y方向大小
  points.color.g = 1.0f;                            // 设置颜色为绿色
  points.color.a = 1.0; // 设置颜色的不透明度为完全不透明

  // 填充轨迹点
  for (size_t i = 0; i < cx.size(); ++i) {
    geometry_msgs::Point p; // 创建一个点
    p.x = cx[i];            // 设置点的X坐标
    p.y = cy[i];            // 设置点的Y坐标
    p.z = 0; // 设置点的Z坐标为0，因为这是一个2D轨迹
    points.points.push_back(p); // 将点添加到Marker的点列表中
  }

  marker_pub.publish(points); // 发布Marker消息
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "mpc_node");
  ros::NodeHandle nh;

  loadVehicleConfig(nh);
  loadMPCConfig(nh);

  ROS_INFO("Configuration loading complete. Node is now spinning.");

  // 参考轨迹
  // std::vector<double> ax = {0.0, 15.0, 30.0, 50.0, 60.0}; // 参考路径的x坐标
  // std::vector<double> ay = {0.0, 40.0, 15.0, 30.0, 0.0}; // 参考路径的y坐标
  std::vector<double> ax = {0.0, 1.0, 2.0, 3.0, 4.0}; // 参考路径的x坐标
  std::vector<double> ay = {0.0, 0.5, 1.0, 0.5, 0.0}; // 参考路径的y坐标
  double ds = 0.01;                                   // 间隔距离

  std::vector<double> cx, cy, cyaw, ck, s;
  std::cout << "calc_spline_course" << std::endl;
  calc_spline_course(ax, ay, ds, cx, cy, cyaw, ck,
                     s); // 计算样条曲线路径

  // 输出结果
  for (size_t i = 0; i < cx.size(); ++i) {
    std::cout << "x: " << cx[i] << ", y: " << cy[i] << ", yaw: " << cyaw[i]
              << ", curvature: " << ck[i] << ", s: " << s[i] << std::endl;
  }
  // 写入.csv文件
  std::ofstream ofs("spline_course.csv");
  ofs << "x, y, yaw, curvature, s" << std::endl;
  for (size_t i = 0; i < cx.size(); ++i) {
    ofs << cx[i] << ", " << cy[i] << ", " << cyaw[i] << ", " << ck[i] << ", "
        << s[i] << std::endl;
  }
  std::vector<double> speed_profile =
      calc_speed_profile(cx, cy, cyaw, 10.0 / 3.6);

  for (size_t i = 0; i < speed_profile.size(); ++i) {
    std::cout << "speed: " << speed_profile[i] << std::endl;
  }

  // 发布轨迹
  ros::Publisher marker_pub =
      nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  // publish_trajectory(cx, cy, marker_pub);
  PATH ref_path(cx, cy, cyaw, ck);

  // 起始点
  Node node(cx[0], cy[0], cyaw[0], 0.0);

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

  // 设置控制循环的频率
  ros::Rate loop_rate(50); // 50 Hz
  while (ros::ok()) {
    publish_trajectory(cx, cy, marker_pub);
    node.x = x_;
    node.y = y_;
    node.yaw = yaw_;
    node.v = v_;

    x.push_back(node.x);
    y.push_back(node.y);
    yaw.push_back(node.yaw);
    v.push_back(node.v);
    t.push_back(time);
    d.push_back(delta_exc_old);
    a.push_back(a_exc_old);
    std::ofstream ofs2("result.csv");
    ofs2 << "x, y, yaw, v, t, delta, a" << std::endl;
    for (size_t i = 0; i < x.size(); ++i) {
      ofs2 << x[i] << ", " << y[i] << ", " << yaw[i] << ", " << v[i] << ", "
           << t[i] << ", " << d[i] << ", " << a[i] << std::endl;
    }
    std::cout << "time: " << time << std::endl;

    int target_ind;
    std::vector<std::vector<double>> ref_pat_T = calc_ref_trajectory_in_T_step(
        node, ref_path, speed_profile, target_ind);
    // 打印ref_pat_T
    for (size_t i = 0; i < ref_pat_T.size(); ++i) {
      for (size_t j = 0; j < ref_pat_T[i].size(); ++j) {
        std::cout << ref_pat_T[i][j] << " ";
      }
      std::cout << std::endl;
    }

    std::vector<double> z0 = {node.x, node.y, node.v, node.yaw};
    linear_mpc_control(ref_pat_T, z0, a_opt, delta_opt, a_exc_old,
                       delta_exc_old);

    if (!delta_opt.empty()) {
      delta_exc_old = delta_opt.back();
      a_exc_old = a_opt.back();
      std::cout << "a_exc_old: " << a_exc_old
                << ", delta_exc_old: " << delta_exc_old << std::endl;
    }
    // 转换为阿克曼命令
    double delta_cmd = delta_opt.back();
    double a_cmd = a_opt.back();
    time += MPCParams_.dt;

    // 发布gazebo控制命令
    //根据接收到的阿克曼驱动命令设置油门和转向角
    double displacement =
        node.v * MPCParams_.dt + 0.5 * a_cmd * MPCParams_.dt * MPCParams_.dt;

    std_msgs::Float64 displacement_cmd;
    std_msgs::Float64 a_acker_cmd;
    displacement_cmd.data = displacement * 100;
    a_acker_cmd.data = delta_cmd;
    // 打印displacement和delta_cmd
    std::cout << "displacement: " << displacement << std::endl;
    std::cout << "a_cmd: " << displacement << std::endl;

    //发布计算得到的油门和转向值到相应的控制器
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
