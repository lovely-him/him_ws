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


UP = "w"
LEFT = "a"
DOWN = "s"
RIGHT = "d"
QUIT = "q"
# 获取变量，从参数服务器中中
max_speed = rospy.get_param("~max_speed", 2.0)
max_angle = rospy.get_param("~max_angle", 0.34)
pub_name  = rospy.get_param("~pub_name", "pwm_input")

class key_class:
    # 状态
    state = [False, False, False, False]
    # python多线程语法，创建一个锁，防止多个程序同时调用打印功能
    # 返回一个对象
    state_lock = Lock()
    # 初始化赋值
    pub_topic = None
    control = False
    root = None
    def __init__(self):
        # 发布 话题 topic
        self.pub_topic = rospy.Publisher(pub_name, Twist, queue_size=1)
        # 创建周期性调用函数“callback”，频率是1/0.1=10Hz，
        rospy.Timer(rospy.Duration(0.005), self.callback)
        # 注册函数。在程序结束时，先注册的后运行 
        atexit.register(self.exit_func)
        os.system("xset r off")
        # 创建窗口对象的背景色
        self.root = Tk()
        # 框架控件；在屏幕上显示一个矩形区域，多用来作为容器
        frame = Frame(self.root, width=100, height=100)
        frame.bind("<KeyPress>", self.keydown)
        frame.bind("<KeyRelease>", self.keyup)
        frame.pack()
        frame.focus_set()
        # 窗口显示
        lab = Label(frame, height=10, width=30,
            text="Focus on this window\nand use the WASD keys\nto drive the car.\n\nPress Q to quit")
        lab.pack()
        # print("Press %c to quit" % QUIT)
        self.root.mainloop()

    # Up -> linear.x = 1.0
    # Down -> linear.x = -1.0
    # Left ->  angular.z = 1.0
    # Right -> angular.z = -1.0

    # 订阅话题的回调函数
    def callback(self):
        with self.state_lock:
            # 创建msg消息
            data = Twist()
            if not self.control:
                return
            # 根据按下的键赋值
            if self.state[0]:
                data.linear.x = max_speed
            elif self.state[2]:
                data.linear.x = -max_speed

            if self.state[1]:
                data.angular.x =max_angle
            elif self.state[3]:
                data.angular.x = -max_angle
            # 话题发布消息
            if self.pub_topic is not None:
                self.pub_topic.publish(data)

    # 检测哪个按键被按下
    def keyeq(self, e, c):
        return e.char == c or e.keysym == c

    # 按键是否被松开
    def keyup(self, e):
        # python语法
        with self.state_lock:
            if self.keyeq(e, UP):
                self.state[0] = False
            elif self.keyeq(e, LEFT):
                self.state[1] = False
            elif self.keyeq(e, DOWN):
                self.state[2] = False
            elif self.keyeq(e, RIGHT):
                self.state[3] = False
            self.control = sum(self.state) > 0

    # 按键是否被按下？
    def keydown(self, e):
        # python语法
        with self.state_lock:
            if self.keyeq(e, QUIT):
                self.shutdown()
            elif self.keyeq(e, UP):
                self.state[0] = True
                self.state[2] = False
            elif self.keyeq(e, LEFT):
                self.state[1] = True
                self.state[3] = False
            elif self.keyeq(e, DOWN):
                self.state[2] = True
                self.state[0] = False
            elif self.keyeq(e, RIGHT):
                self.state[3] = True
                self.state[1] = False
            self.control = sum(self.state) > 0

    def exit_func(self):
        # system函数可以将字符串转化成命令在服务器上运行；
        os.system("xset r on")

    # 关闭窗口
    def shutdown(self):
        self.root.destroy()
        # destroy()只是终止mainloop并删除所有小部件
        rospy.signal_shutdown("shutdown")

# 主函数
def main():
    rospy.init_node("control_getkey", disable_signals=True)
    key_class()
    # 安全操作
    # signal.signal(signal.SIGINT, lambda s, f: key.shutdown())


if __name__ == "__main__":
    main()