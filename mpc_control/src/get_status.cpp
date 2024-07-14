#include <cmath> // 用于计算数学函数
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

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

  // 延时0.5秒
  ros::Duration(1).sleep();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "odom_listener"); // 初始化ROS节点
  ros::NodeHandle nh;                     // 创建节点句柄

  ros::Subscriber sub =
      nh.subscribe("/odom", 10, odomCallback); // 订阅/odom话题

  ros::spin(); // 保持程序运行直到被关闭
  return 0;
}
