#!/usr/bin/env python3
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
import sys, select, termios, tty

banner = """
1. 从键盘读取输入
2. 发布到 AckermannDriveStamped!
---------------------------
        w
   a    s    d
其他任意键 : 停止
CTRL-C 退出
"""

keyBindings = {
  'w':(1,0),  # 向前
  'd':(1,-1), # 向前并向右转
  'a':(1,1),  # 向前并向左转
  's':(-1,0), # 向后
}

def getKey():
   tty.setraw(sys.stdin.fileno())  # 设置终端为原始模式
   select.select([sys.stdin], [], [], 0)  # 非阻塞式等待输入
   key = sys.stdin.read(1)  # 读取单个字符
   termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)  # 还原终端设置
   return key

speed = 1  # 速度设定
turn = 0.6  # 转向角度设定

def vels(speed,turn):
  return "当前速度: \tspeed %s\t转向 %s " % (speed,turn)  # 返回当前的速度和转向状态

if __name__=="__main__":
  settings = termios.tcgetattr(sys.stdin)  # 获取当前终端的属性设置
  pub = rospy.Publisher("/ackermann_cmd_mux/output", AckermannDriveStamped,queue_size=1)  # 初始化 ROS 发布者
  rospy.init_node('keyop')  # 初始化 ROS 节点

  x = 0  # 线速度
  th = 0  # 转向角
  status = 0  # 状态

  try:
    while(1):
       key = getKey()  # 获取按键
       if key in keyBindings.keys():
          x = keyBindings[key][0]  # 设置线速度
          th = keyBindings[key][1]  # 设置转向角
       else:
          x = 0
          th = 0
          if (key == '\x03'):  # 检测 Ctrl-C
             break
       msg = AckermannDriveStamped()  # 创建消息
       msg.header.stamp = rospy.Time.now()  # 设置时间戳
       msg.header.frame_id = "base_link"  # 设置 frame id

       msg.drive.speed = x*speed  # 设置速度
       msg.drive.acceleration = 1  # 设置加速度
       msg.drive.jerk = 1  # 设置急动
       msg.drive.steering_angle = th*turn  # 设置转向角
       msg.drive.steering_angle_velocity = 1  # 设置转向角速度

       pub.publish(msg)  # 发布消息

  except:
    print('发生错误')

  finally:
    msg = AckermannDriveStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "base_link"

    msg.drive.speed = 0  # 停止
    msg.drive.acceleration = 1
    msg.drive.jerk = 1
    msg.drive.steering_angle = 0
    msg.drive.steering_angle_velocity = 1
    pub.publish(msg)  # 发布停车消息

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)  # 恢复终端设置
