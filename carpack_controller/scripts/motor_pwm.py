#!/usr/bin/env python
#-*-coding:utf-8-*-

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist


old_speed   = 0
old_angle   = 0
car_name    = rospy.get_param('~car_name',"racecar")
sub_name    = rospy.get_param('~sub_name',"pwm_input")
# 左后轮速度控制器
LRW_topic   = '/{}/left_rear_wheel_velocity_controller/command'.format(car_name)
RRW_topic   = '/{}/right_rear_wheel_velocity_controller/command'.format(car_name)
# 左前轮速度控制器
LFW_topic   = '/{}/left_front_wheel_velocity_controller/command'.format(car_name)
RFW_topic   = '/{}/right_front_wheel_velocity_controller/command'.format(car_name)
# 左转向铰链位置控制器
LSH_topic   = '/{}/left_steering_hinge_position_controller/command'.format(car_name)
RSH_topic   = '/{}/right_steering_hinge_position_controller/command'.format(car_name)

class car_pwm:
    # 定义 变量
    min_speed       = -10.0
    max_speed       = 10.0    # 100.0
    min_angle       = -3.14/2
    max_angle       = 3.14/2
    # 最大速度变化率
    speed_alter     = 0.1     # 1.25
    angle_alter     = 0.02     
    def __init__(self):
        # 创建 话题 topic
        self.pub_vel_LRW = rospy.Publisher(LRW_topic, Float64, queue_size = 1)
        self.pub_vel_RRW = rospy.Publisher(RRW_topic, Float64, queue_size = 1)
        self.pub_vel_LFW = rospy.Publisher(LFW_topic, Float64, queue_size = 1)
        self.pub_vel_RFW = rospy.Publisher(RFW_topic, Float64, queue_size = 1)
        self.pub_pos_LSH = rospy.Publisher(LSH_topic, Float64, queue_size = 1)
        self.pub_pos_RSH = rospy.Publisher(RSH_topic, Float64, queue_size = 1)
        # 订阅 话题 topic
        rospy.Subscriber(sub_name, Twist, self.callback, queue_size=1)

    def callback(self, data):
        global old_speed
        global old_angle
        # 创建 msg 消息, 注意:ros的float64是一个结构体
        angle = Float64()
        speed = Float64()
        # 提取 线速度 与 速度
        speed.data = ((data.linear.x) * 8)#30
        angle.data = ((data.angular.z) * 1)#16
        # 保存变化的方向
        if speed.data < old_speed:
            speed_dir = -1.0
        else:
            speed_dir = 1.0
        if speed.data < 0:
            speed_dir2 = -1.0
        else:
            speed_dir2 = 1.0
        if angle.data < old_angle:
            angle_dir = -1.0
        else:
            angle_dir = 1.0
        # 限制变化率
        #if abs(speed.data - old_speed) > self.speed_alter:
        #    speed.data = old_speed + self.speed_alter * speed_dir
        #if abs(angle.data - old_angle) > self.angle_alter:
        #    angle.data = old_angle + self.angle_alter * angle_dir
        # 限幅
        if speed.data > self.max_speed:
            speed.data = self.max_speed
        elif speed.data < self.min_speed:
            speed.data = self.min_speed
        if angle.data > self.max_angle:
            angle.data = self.max_angle
        elif angle.data < self.min_angle:
            angle.data = self.min_angle
        # 保存上一次的值
        old_speed = speed.data
        old_angle = angle.data
        # 差速转向
        speed_angle_L = speed.data# - speed_dir2*angle.data*10
        speed_angle_R = speed.data# + speed_dir2*angle.data*10
        # 左后轮速度控制器
        self.pub_vel_LRW.publish(speed.data)
        self.pub_vel_RRW.publish(speed.data)
        # 左前轮速度控制器
        self.pub_vel_LFW.publish(speed.data)
        self.pub_vel_RFW.publish(speed.data)
        # 左转向铰链位置控制器
        self.pub_pos_LSH.publish(angle.data)
        self.pub_pos_RSH.publish(angle.data)

def main():
    rospy.init_node("motor_pwm")
    car_pwm()
    rospy.spin()

if __name__ == '__main__':
    main()
