#include <ros/ros.h>
#include <std_msgs/Float64.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "racebot_controller");
  ros::NodeHandle nh;

  // 定义发布者
  ros::Publisher pub_vel_left_rear_wheel = nh.advertise<std_msgs::Float64>(
      "/racebot/left_rear_wheel_velocity_controller/command", 1);
  ros::Publisher pub_vel_right_rear_wheel = nh.advertise<std_msgs::Float64>(
      "/racebot/right_rear_wheel_velocity_controller/command", 1);
  ros::Publisher pub_vel_left_front_wheel = nh.advertise<std_msgs::Float64>(
      "/racebot/left_front_wheel_velocity_controller/command", 1);
  ros::Publisher pub_vel_right_front_wheel = nh.advertise<std_msgs::Float64>(
      "/racebot/right_front_wheel_velocity_controller/command", 1);
  ros::Publisher pub_pos_left_steering_hinge = nh.advertise<std_msgs::Float64>(
      "/racebot/left_steering_hinge_position_controller/command", 1);
  ros::Publisher pub_pos_right_steering_hinge = nh.advertise<std_msgs::Float64>(
      "/racebot/right_steering_hinge_position_controller/command", 1);

  ros::Rate loop_rate(10); // 10 Hz

  while (ros::ok()) {
    // 创建消息
    std_msgs::Float64 left_rear_wheel_speed;
    std_msgs::Float64 right_rear_wheel_speed;
    std_msgs::Float64 left_front_wheel_speed;
    std_msgs::Float64 right_front_wheel_speed;
    std_msgs::Float64 left_steering_angle;
    std_msgs::Float64 right_steering_angle;

    // 设置速度和转向角度
    left_rear_wheel_speed.data = 1.0;   // 设置为1.0 m/s
    right_rear_wheel_speed.data = 1.0;  // 设置为1.0 m/s
    left_front_wheel_speed.data = 1.0;  // 设置为1.0 m/s
    right_front_wheel_speed.data = 1.0; // 设置为1.0 m/s
    left_steering_angle.data = 0.0;     // 设置转向角度为0度（直行）
    right_steering_angle.data = 0.0;    // 设置转向角度为0度（直行）

    // 发布消息
    pub_vel_left_rear_wheel.publish(left_rear_wheel_speed);
    pub_vel_right_rear_wheel.publish(right_rear_wheel_speed);
    pub_vel_left_front_wheel.publish(left_front_wheel_speed);
    pub_vel_right_front_wheel.publish(right_front_wheel_speed);
    pub_pos_left_steering_hinge.publish(left_steering_angle);
    pub_pos_right_steering_hinge.publish(right_steering_angle);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
