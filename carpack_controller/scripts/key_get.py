#!/usr/bin/env python
#-*-coding:utf-8-*-

# Copyright (c) 2019, The Personal Robotics Lab, The MuSHR Team, The Contributors of MuSHR
# License: BSD 3-Clause. See LICENSE.md file in root directory.

import atexit
import os
import signal
from threading import Lock
from Tkinter import Frame, Label, Tk

import rospy
from geometry_msgs.msg import Twist

# 定义全局变量
UP = "w"
LEFT = "a"
DOWN = "s"
RIGHT = "d"
QUIT = "q"
key_pass = rospy.get_param('~key_pass',0)
# 状态
state = [False, False, False, False]
# python多线程语法，创建一个锁，防止多个程序同时调用打印功能
state_lock = Lock()
# 初始化赋值
state_pub = None
root = None
control = False

# 检测哪个按键被按下
def keyeq(e, c):
    return e.char == c or e.keysym == c

# 按键是否被松开
def keyup(e):
    global state
    global control
    # python语法
    with state_lock:
        if keyeq(e, UP):
            state[0] = False
        elif keyeq(e, LEFT):
            state[1] = False
        elif keyeq(e, DOWN):
            state[2] = False
        elif keyeq(e, RIGHT):
            state[3] = False
        control = sum(state) > 0

# 按键是否被按下？
def keydown(e):
    global state
    global control
    # python语法
    with state_lock:
        if keyeq(e, QUIT):
            shutdown()
        elif keyeq(e, UP):
            state[0] = True
            state[2] = False
        elif keyeq(e, LEFT):
            state[1] = True
            state[3] = False
        elif keyeq(e, DOWN):
            state[2] = True
            state[0] = False
        elif keyeq(e, RIGHT):
            state[3] = True
            state[1] = False
        control = sum(state) > 0


# Up -> linear.x = 1.0
# Down -> linear.x = -1.0
# Left ->  angular.z = 1.0
# Right -> angular.z = -1.0

# 订阅话题的回调函数
def publish_cb(_):
    global key_pass
    # 创建msg消息
    ack = Twist()
    with state_lock:
        if not control:
            if key_pass:
                ack.linear.x = 0
                ack.angular.z = 0
                # 话题发布消息
                if state_pub is not None:
                    state_pub.publish(ack)
            return
        # 根据按下的键赋值
        if state[0]:
            ack.linear.x = max_velocity
        elif state[2]:
            ack.linear.x = -max_velocity

        if state[1]:
            ack.angular.z = max_steering_angle
        elif state[3]:
            ack.angular.z = -max_steering_angle
        # 话题发布消息
        if state_pub is not None:
            state_pub.publish(ack)


def exit_func():
    # system函数可以将字符串转化成命令在服务器上运行；
    os.system("xset r on")


def shutdown():
    root.destroy()
    # destroy()只是终止mainloop并删除所有小部件
    rospy.signal_shutdown("shutdown")

# 主函数
def main():
    global state_pub
    global root
    # 声明全局变量
    global max_velocity
    global max_steering_angle
    # 获取变量，从参数服务器中中
    max_velocity = rospy.get_param("~max_speed", 6.0)
    max_steering_angle = rospy.get_param("~max_angle", 0.34)
    # 不设立默认值了，确保有写
    key_publisher = rospy.get_param("~pub_name", "pwm_input")
    # 创建topic话题，发送按键信息
    state_pub = rospy.Publisher(key_publisher, Twist, queue_size=1)
    # 创建周期性调用函数“publish_cb”，频率是1/0.1=10Hz，
    rospy.Timer(rospy.Duration(0.005), publish_cb)
    # 注册函数。在程序结束时，先注册的后运行 
    atexit.register(exit_func)
    os.system("xset r off")
    # 创建窗口对象的背景色
    root = Tk()
    # 框架控件；在屏幕上显示一个矩形区域，多用来作为容器
    frame = Frame(root, width=100, height=100)
    frame.bind("<KeyPress>", keydown)
    frame.bind("<KeyRelease>", keyup)
    frame.pack()
    frame.focus_set()
    # 窗口显示
    lab = Label(
        frame,
        height=10,
        width=30,
        text="Focus on this window\nand use the WASD keys\nto drive the car.\n\nPress Q to quit",
    )
    lab.pack()
    print("Press %c to quit" % QUIT)
    root.mainloop()


if __name__ == "__main__":
    rospy.init_node("key_get", disable_signals=True)
    # 安全操作
    signal.signal(signal.SIGINT, lambda s, f: shutdown())
    main()