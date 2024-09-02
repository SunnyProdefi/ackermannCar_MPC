// LQR.h
#ifndef LQR_H
#define LQR_H

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <vector>

// Constants
constexpr double PI = 3.14159265358979323846;
constexpr double DEG2RAD = PI / 180.0;
constexpr double ts = 0.1;
constexpr double l_f = 1.165 / 10;
constexpr double l_r = 1.165 / 10;
constexpr double max_acceleration = 5.0;         // [m/s^2]
constexpr double max_steer_angle = 40 * DEG2RAD; // [rad]
constexpr double max_speed = 35 / 3.6;
constexpr double max_iteration = 200;
constexpr int state_size = 4; // [m/s]

enum class Gear { DRIVE, REVERSE };

class VehicleState {
public:
  double x, y, yaw, v, e_cg, theta_e, steer;
  Gear gear;

  VehicleState(double x = 0.0, double y = 0.0, double yaw = 0.0, double v = 0.0,
               Gear gear = Gear::DRIVE);

  void updateVehicleState(double delta, double a, double e_cg, double theta_e,
                          Gear gear);
  static void regulateInput(double &delta, double &a);
  static double regulateOutput(double v);
};

double normalizeAngle(double angle);

class TrajectoryAnalyzer {
public:
  std::vector<double> x_, y_, yaw_, k_;
  size_t ind_old, ind_end;

  TrajectoryAnalyzer(const std::vector<double> &x, const std::vector<double> &y,
                     const std::vector<double> &yaw,
                     const std::vector<double> &k);

  void toTrajectoryFrame(const VehicleState &vehicle_state);
};

class LatController {
public:
  Eigen::MatrixXd matrix_q;
  Eigen::MatrixXd matrix_r;

  LatController();
  std::tuple<double, double, double>
  computeControlCommand(const VehicleState &vehicle_state,
                        const TrajectoryAnalyzer &ref_trajectory);
  double computeFeedForward(double ref_curvature);
  Eigen::MatrixXd solveLQRProblem(const Eigen::MatrixXd &A,
                                  const Eigen::MatrixXd &B,
                                  const Eigen::MatrixXd &Q,
                                  const Eigen::MatrixXd &R, double tolerance,
                                  int max_num_iteration);
  std::tuple<Eigen::MatrixXd, Eigen::MatrixXd>
  updateMatrix(const VehicleState &vehicle_state);
};

class LonController {
public:
  static double computeControlCommand(double target_speed,
                                      const VehicleState &vehicle_state,
                                      double dist);
};

class PathGenerator {
public:
  // Generates a sinusoidal path given amplitude, frequency, path length, and
  // step size
  static std::tuple<std::vector<double>, std::vector<double>,
                    std::vector<double>, std::vector<double>,
                    std::vector<double>>
  generateSineWavePath(double amplitude, double frequency, double length,
                       double step);
};

#endif // LQR_H
