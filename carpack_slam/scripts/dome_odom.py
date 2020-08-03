#!/usr/bin/env python
#-*-coding:utf-8-*-

'''
This script makes Gazebo less fail by translating gazebo status messages to odometry data.
Since Gazebo also publishes data faster than normal odom data, this script caps the update to 20hz.
Winter Guerra
'''

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Transform, TransformStamped
from gazebo_msgs.msg import LinkStates
from std_msgs.msg import Header
import numpy as np
import math
import tf2_ros

class OdometryNode:
    # Set publishers 设置发布话题
    pub_odom = rospy.Publisher('/odom_him', Odometry, queue_size=1)

    def __init__(self):
        # init internals 初始化内部
        self.last_received_pose = Pose()
        self.last_received_twist = Twist()
        self.last_recieved_stamp = None

        # Set subscribers 设置订阅话题
        rospy.Subscriber('/gazebo/link_states', LinkStates, self.sub_robot_pose_update)

    def sub_robot_pose_update(self, msg):
        # Find the index of the racecar 找到赛车的索引
        try:
            #创建msg消息对象
            arrayIndex = msg.name.index('racecar::base_footprint')
        except ValueError as e:
            # Wait for Gazebo to startup 等待Gazebo启动
            pass
        else:
            # Extract our current position information 提取我们当前的位置信息
            self.last_received_pose = msg.pose[arrayIndex]
            self.last_received_twist = msg.twist[arrayIndex]
        self.last_recieved_stamp = rospy.Time.now()
        #print(self.last_received_twist.linear.x)
        #print(self.last_received_pose.position.x)
        #print("-----")

# Start the node
if __name__ == '__main__':
    rospy.init_node("mapping_odometry")#建立节点
    #调用类。在类调用类的同时调用了发送tf和topic的函数
    node = OdometryNode()
    #等待阻塞
    rospy.spin()