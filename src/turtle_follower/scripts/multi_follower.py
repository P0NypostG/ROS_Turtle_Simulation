#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import subprocess
import time
import sys

def launch_followers(num_followers=3):
    """启动多个跟随者"""
    processes = []
    
    # 启动roscore（如果还没启动）
    try:
        rospy.init_node('multi_follower_launcher', anonymous=True)
    except:
        pass
    
    # 启动turtlesim
    rospy.loginfo("启动turtlesim模拟器...")
    turtlesim_proc = subprocess.Popen(['rosrun', 'turtlesim', 'turtlesim_node'])
    processes.append(turtlesim_proc)
    time.sleep(2)
    
    # 启动leader
    rospy.loginfo("启动Leader...")
    leader_proc = subprocess.Popen(['rosrun', 'turtle_follower', 'leader.py'])
    processes.append(leader_proc)
    time.sleep(2)
    
    # 启动多个follower（链式跟随）
    followers = []
    for i in range(1, num_followers + 1):
        robot_name = f'turtle{i+1}'  # turtle2, turtle3, ...
        
        # 设置跟随目标：第一个跟随turtle1，后面的跟随前一个
        if i == 1:
            target_robot = 'turtle1'
        else:
            target_robot = f'turtle{i}'
        
        rospy.loginfo(f"启动 {robot_name}，跟随 {target_robot}...")
        
        # 启动follower进程
        follower_proc = subprocess.Popen([
            'rosrun', 'turtle_follower', 
            'follower_chain.py',  # 新的链式跟随脚本
            robot_name,
            target_robot,
            str(3.0 + i * 2.0),  # 初始X位置
            str(3.0 + i * 2.0),  # 初始Y位置
            '1.0'  # 跟随距离
        ])
        processes.append(follower_proc)
        followers.append(robot_name)
        time.sleep(1)
    
    rospy.loginfo(f"已启动 {num_followers} 个跟随者: {followers}")
    
    try:
        # 保持程序运行
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        rospy.loginfo("正在关闭所有进程...")
        for proc in processes:
            proc.terminate()
        for proc in processes:
            proc.wait()
        rospy.loginfo("所有进程已关闭")

if __name__ == '__main__':
    # 从命令行参数获取跟随者数量
    num_followers = 3
    if len(sys.argv) > 1:
        num_followers = int(sys.argv[1])
    
    launch_followers(num_followers)
