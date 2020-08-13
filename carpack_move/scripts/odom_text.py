#!/usr/bin/env python
#-*-coding:utf-8-*-

import rospy
import math
import tf2_ros
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


pub_name    = "odom_error"
sub_name    = 'odom_now'
sub2_name   = "odom_topic"

class odom_class:
    vesc_odom  = Odometry()
    # 初始化
    def __init__(self):
        # 订阅 odom_gazebo
        rospy.Subscriber(sub_name, Odometry, self.callback_imu, queue_size=1)
        # 订阅 odom_imu
        rospy.Subscriber(sub2_name, Odometry, self.callback_gazebo, queue_size=1)
        # 发布 odom_error
        self.pub = rospy.Publisher(pub_name, Odometry, queue_size = 1)
        # 定义时间差
        rate = rospy.Rate(20)

    # 获取 gazebo 发布的 odom 坐标
    def callback_gazebo(self, data):
        self.vesc_odom = data
        # 偏航角 误差，使用四元数最后一位记录
        yaw_angle = euler_from_quaternion(
            (data.pose.pose.orientation.x,data.pose.pose.orientation.y,
             data.pose.pose.orientation.z,data.pose.pose.orientation.w))
        self.vesc_odom.pose.pose.orientation.w = yaw_angle[2]

    # 获取 imu 计算的 odom 坐标
    def callback_imu(self, data):
        error = Odometry()
        # 记录基本消息
        error.header.stamp    = rospy.Time.now()
        error.header.frame_id = sub_name
        error.child_frame_id  = sub2_name
        # xy轴坐标 误差
        error.pose.pose.position.x = self.vesc_odom.pose.pose.position.x - data.pose.pose.position.x 
        error.pose.pose.position.y = self.vesc_odom.pose.pose.position.y - data.pose.pose.position.y 
        error.pose.pose.position.z = 0 
        # xy轴线速度 误差
        error.twist.twist.linear.x = self.vesc_odom.twist.twist.linear.x - data.twist.twist.linear.x 
        error.twist.twist.linear.y = self.vesc_odom.twist.twist.linear.y - data.twist.twist.linear.y  
        error.twist.twist.linear.z = 0 
        # 偏航角 误差，使用四元数最后一位记录
        yaw_angle = euler_from_quaternion(
            (data.pose.pose.orientation.x,data.pose.pose.orientation.y,
             data.pose.pose.orientation.z,data.pose.pose.orientation.w))
        data.pose.pose.orientation.w  = yaw_angle[2]
        error.pose.pose.orientation.w = self.vesc_odom.pose.pose.orientation.w - data.pose.pose.orientation.w
        # 其他用不到就不管了
        # 发布出去
        self.pub.publish(error)

        
        
def main():
    rospy.init_node("odom_text")
    node = odom_class()
    rospy.spin()

if __name__ == '__main__':
    main()