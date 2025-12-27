#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn
import math

class IntruderGang:
    def __init__(self):
        rospy.init_node('gang_node', anonymous=True)

        # === 1. 生成海龟 ===
        self.spawn_turtles()

        # === 2. 通信设置 ===
        # 订阅位姿
        self.pose_t1 = Pose() # 警卫
        self.pose_t2 = Pose() # 老大 (你控制)
        self.pose_t3 = Pose() # 小弟 (自动跟随)
        
        rospy.Subscriber('/turtle1/pose', Pose, self.cb_t1)
        rospy.Subscriber('/turtle2/pose', Pose, self.cb_t2)
        rospy.Subscriber('/turtle3/pose', Pose, self.cb_t3)

        # 发布小弟的控制指令，不发布老大的
        self.pub_t3 = rospy.Publisher('/turtle3/cmd_vel', Twist, queue_size=10)

        # === 3. 逻辑参数 ===
        self.panic_distance = 1.0  # 恐慌阈值
        self.is_panic = False

        rospy.loginfo("团伙逻辑已启动：请使用键盘控制 Turtle2 (老大)，Turtle3 (小弟) 会自动跟随")

    def spawn_turtles(self):
        rospy.wait_for_service('spawn')
        spawner = rospy.ServiceProxy('spawn', Spawn)
        try:
            spawner(5.0, 5.0, 0, 'turtle2')
            spawner(4.0, 4.0, 0, 'turtle3')
        except rospy.ServiceException:
            pass

    def cb_t1(self, data): self.pose_t1 = data
    def cb_t2(self, data): self.pose_t2 = data
    def cb_t3(self, data): self.pose_t3 = data


    def control_minion(self):
        """小弟逻辑：跟随头目 OR 恐慌逃跑"""
        cmd = Twist()
        
        # 计算警卫到老大的距离
        dist_guard_to_boss = math.sqrt((self.pose_t1.x - self.pose_t2.x)**2 + 
                                       (self.pose_t1.y - self.pose_t2.y)**2)

        # 判定恐慌
        if dist_guard_to_boss < self.panic_distance:
            self.is_panic = True
            rospy.logwarn_throttle(1, "!!! 老大被抓了！快跑！(PANIC MODE) !!!")
        else:
            if dist_guard_to_boss > self.panic_distance * 2:
                self.is_panic = False

        if self.is_panic:
            # === 逃跑逻辑 ===
            dx = self.pose_t3.x - self.pose_t1.x
            dy = self.pose_t3.y - self.pose_t1.y
            escape_angle = math.atan2(dy, dx)
            
            angle_error = escape_angle - self.pose_t3.theta
            while angle_error > math.pi: angle_error -= 2*math.pi
            while angle_error < -math.pi: angle_error += 2*math.pi
            
            cmd.linear.x = 2.5
            cmd.angular.z = 5.0 * angle_error
        
        else:
            # === 跟随逻辑 (跟随老大) ===
            dx = self.pose_t2.x - self.pose_t3.x
            dy = self.pose_t2.y - self.pose_t3.y
            target_angle = math.atan2(dy, dx)
            dist_to_boss = math.sqrt(dx**2 + dy**2)

            angle_error = target_angle - self.pose_t3.theta
            while angle_error > math.pi: angle_error -= 2*math.pi
            while angle_error < -math.pi: angle_error += 2*math.pi

            if dist_to_boss > 0.8:
                cmd.linear.x = 1.5 * (dist_to_boss - 0.8)
                cmd.angular.z = 4.0 * angle_error
            else:
                cmd.linear.x = 0
                cmd.angular.z = 4.0 * angle_error

        return cmd

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            # 只控制小弟
            self.pub_t3.publish(self.control_minion())
            rate.sleep()

if __name__ == '__main__':
    try:
        gang = IntruderGang()
        gang.run()
    except rospy.ROSInterruptException:
        pass
