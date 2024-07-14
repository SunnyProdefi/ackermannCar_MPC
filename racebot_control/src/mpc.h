#ifndef MPC_H
#define MPC_H

#include <vector>

// 定义结构体
struct MPCParams {
  int NX; // 状态向量的维数: z = [x, y, v, phi]
  int NU; // 输入向量的维数: u = [加速度, 转向角]
  int T;  // 预测步数
  std::vector<std::vector<double>> Q;  // 状态权重
  std::vector<std::vector<double>> Qf; // 终端状态权重
  std::vector<std::vector<double>> R;  // 输入权重
  std::vector<std::vector<double>> Rd; // 输入变化率权重(未使用，以约束代替)
  double dist_stop;                    // 停止距离
  double speed_stop;                   // 停止速度
  double time_max;                     // 最大时间
  int iter_max;                        // 最大迭代次数
  double target_speed;                 // 目标速度
  int N_IND;                           // 搜索索引数量
  double dt;                           // 时间间隔
  int d_dist;                          // 距离步长
  double du_res;                       //停止迭代的阈值
};

struct vehicleParams {
  double RF;               // 前轮半径
  double RB;               // 后轮半径
  double W;                // 车宽
  double WD;               // 驱动轮距
  double WB;               // 轴距
  double TR;               // 轮胎半径
  double TW;               // 轮胎宽度
  double steer_max;        // 最大转向角
  double steer_change_max; // 最大转向角变化率
  double speed_max;        // 最大速度
  double speed_min;        // 最小速度
  double acceleration_max; // 最大加速度
};

#endif // MPC_H
