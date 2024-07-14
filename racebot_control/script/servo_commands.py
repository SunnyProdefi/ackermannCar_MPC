#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from ackermann_msgs.msg import AckermannDriveStamped

flag_move = 0  # 定义一个移动标志，初始设置为0，此脚本中未使用此变量

# 定义设置油门和转向的函数
def set_throttle_steer(data):

    global flag_move  # 引用全局变量flag_move

    # 定义并初始化各个车轮和转向铰链的速度/位置控制器的发布者
    pub_vel_left_rear_wheel = rospy.Publisher('/racebot/left_rear_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_right_rear_wheel = rospy.Publisher('/racebot/right_rear_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_left_front_wheel = rospy.Publisher('/racebot/left_front_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_right_front_wheel = rospy.Publisher('/racebot/right_front_wheel_velocity_controller/command', Float64, queue_size=1)

    pub_pos_left_steering_hinge = rospy.Publisher('/racebot/left_steering_hinge_position_controller/command', Float64, queue_size=1)
    pub_pos_right_steering_hinge = rospy.Publisher('/racebot/right_steering_hinge_position_controller/command', Float64, queue_size=1)

    # 根据接收到的阿克曼驱动命令设置油门和转向角
    throttle = data.drive.speed * 31.25  # 计算油门值
    steer = data.drive.steering_angle  # 获取转向角

    # 发布计算得到的油门和转向值到相应的控制器
    pub_vel_left_rear_wheel.publish(throttle)
    pub_vel_right_rear_wheel.publish(throttle)
    pub_vel_left_front_wheel.publish(throttle)
    pub_vel_right_front_wheel.publish(throttle)
    pub_pos_left_steering_hinge.publish(steer)
    pub_pos_right_steering_hinge.publish(steer)

# 定义主函数，初始化ROS节点，并订阅阿克曼驱动命令
def servo_commands():

    rospy.init_node('servo_commands', anonymous=True)  # 初始化ROS节点

    rospy.Subscriber("/ackermann_cmd_mux/output", AckermannDriveStamped, set_throttle_steer)  # 订阅阿克曼驱动命令

    # spin() 保持python运行直至节点被关闭
    rospy.spin()

if __name__ == '__main__':
    try:
        servo_commands()  # 调用主函数
    except rospy.ROSInterruptException:
        pass  # 在ROS中断异常发生时，什么也不做，正常退出程序
