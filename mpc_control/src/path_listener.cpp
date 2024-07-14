#include <nav_msgs/Path.h>
#include <ros/ros.h>

void pathCallback(const nav_msgs::Path::ConstPtr &msg) {
  ROS_INFO("Received a new path!");
  for (const auto &pose : msg->poses) {
    ROS_INFO("Pose: (%f, %f, %f)", pose.pose.position.x, pose.pose.position.y,
             pose.pose.orientation.z);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "path_listener");
  ros::NodeHandle nh;

  ros::Subscriber sub =
      nh.subscribe("move_base/NavfnROS/plan", 1000, pathCallback);

  ros::spin();
  return 0;
}
