#!/usr/bin/env python3
import rospy  # 导入rospy，用于编写ROS节点
import std_msgs.msg  # 导入标准消息类型
from ackermann_msgs.msg import AckermannDriveStamped  # 导入Ackermann驱动消息类型
from geometry_msgs.msg import Twist  # 导入Twist消息类型，常用于机器人速度和旋转的表示

import time  # 导入time模块
import threading  # 导入threading模块，用于多线程
pub = rospy.Publisher("/ackermann_cmd_mux/output", AckermannDriveStamped, queue_size=1)  # 创建一个Publisher，发布AckermannDriveStamped类型的消息

def thread_job():
    rospy.spin()  # 保持节点运行，监听回调函数

def callback(data):
    speed = data.linear.x  # 从Twist消息中获取线速度
    turn = data.angular.z  # 从Twist消息中获取角速度

    msg = AckermannDriveStamped()  # 创建一个AckermannDriveStamped消息
    msg.header.stamp = rospy.Time.now()  # 设置消息时间戳
    msg.header.frame_id = "base_link"  # 设置消息的frame_id

    msg.drive.speed = speed  # 设置行驶速度
    msg.drive.acceleration = 1  # 设置加速度
    msg.drive.jerk = 1  # 设置冲击
    msg.drive.steering_angle = turn  # 设置转向角度
    msg.drive.steering_angle_velocity = 1  # 设置转向角速度

    pub.publish(msg)  # 发布消息

def SubscribeAndPublish():
    rospy.init_node('nav_sim', anonymous=True)  # 初始化节点
    rospy.Subscriber('cmd_vel', Twist, callback, queue_size=1, buff_size=52428800)  # 订阅cmd_vel主题
    rospy.spin()  # 保持节点运行，监听回调函数


if __name__ == '__main__':
    try:
        SubscribeAndPublish()  # 执行主函数
    except rospy.ROSInterruptException:
        pass  # 如果有异常发生，则忽略
