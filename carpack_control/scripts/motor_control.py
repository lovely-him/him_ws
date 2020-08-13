#!/usr/bin/env python
#-*-coding:utf-8-*-

# 加载ros的Python基础包
import rospy
# 加载topic话题 的 msg消息
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

class main_class:
    # 初始化函数
    def __init__(self):
        # 创建node节点 —— 电机控制
        rospy.init_node('motor_control', anonymous=True)
        # 订阅topic话题 —— 电机pwm输出
        rospy.Subscriber("motor_output", Twist, self.callback)
        # 发布topic话题 —— 线速度输出
        self.pub_lrw = rospy.Publisher('/racecar/left_rear_wheel_velocity_controller/command', Float64, queue_size=10)
        self.pub_rrw = rospy.Publisher('/racecar/right_rear_wheel_velocity_controller/command', Float64, queue_size=10)
        self.pub_lfw = rospy.Publisher('/racecar/left_front_wheel_velocity_controller/command', Float64, queue_size=10)
        self.pub_rfw = rospy.Publisher('/racecar/right_front_wheel_velocity_controller/command', Float64, queue_size=10)
        # 发布topic话题 —— 角速度输出
        self.pub_lsh = rospy.Publisher('/racecar/left_steering_hinge_position_controller/command', Float64, queue_size=10)
        self.pub_rsh = rospy.Publisher('/racecar/right_steering_hinge_position_controller/command', Float64, queue_size=10)
        # 阻塞等待
        rospy.spin()
    # 回调函数
    def callback(self,data):
        # 创建 msg 消息, 注意:ros的float64是一个结构体
        angle = Float64()
        speed = Float64()
        # 提取 线速度 与 角速度
        speed.data = ((data.linear.x) * 8)
        angle.data = ((data.angular.z) * 1)
        # 向topic话题 发送 msg消息
        self.pub_lrw.publish(speed.data)
        self.pub_rrw.publish(speed.data)
        self.pub_lfw.publish(speed.data)
        self.pub_rfw.publish(speed.data)
        self.pub_lsh.publish(angle.data)
        self.pub_rsh.publish(angle.data)

if __name__ == '__main__':
    try:
        main_class()
    except rospy.ROSInterruptException:
        pass
