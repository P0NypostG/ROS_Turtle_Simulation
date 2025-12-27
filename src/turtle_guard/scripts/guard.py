#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn
from std_msgs.msg import Float64 
import math

# 导入动态配置模块
from dynamic_reconfigure.server import Server
from turtle_guard.cfg import GuardConfigConfig

class TurtleGuard:
    def __init__(self):
        rospy.init_node('guard_turtle', anonymous=True)
        # 注意：这里不再负责生成 turtle2/3，交给 gang.py 处理
        
        self.guard_pose = Pose()
        self.t2_pose = Pose() # 头目
        self.t3_pose = Pose() # 小弟
        
        rospy.Subscriber('/turtle1/pose', Pose, self.cb_guard)
        rospy.Subscriber('/turtle2/pose', Pose, self.cb_t2)
        rospy.Subscriber('/turtle3/pose', Pose, self.cb_t3) # 新增订阅

        self.pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.dist_pub = rospy.Publisher('/guard/min_dist', Float64, queue_size=10)

        # (参数部分和 dynamic_reconfigure 部分保持不变，请保留)
        self.state = "PATROL"
        self.detection_radius = 3.0
        self.waypoints = [(2, 2), (9, 2), (9, 9), (2, 9)]
        self.current_wp_index = 0
        self.wp_tolerance = 0.5
        self.kp_linear = 1.5
        self.kp_angular = 4.0
        self.srv = Server(GuardConfigConfig, self.reconfigure_callback)

        rospy.loginfo("警卫系统就绪！")

    def reconfigure_callback(self, config, level):
        """当 rqt_reconfigure 滑块拖动时触发"""
        self.kp_linear = config.kp_linear
        self.kp_angular = config.kp_angular
        self.detection_radius = config.detection_radius
        self.wp_tolerance = config.wp_tolerance
        rospy.loginfo("参数更新 -> P_Lin: %.2f, Radius: %.2f" % (self.kp_linear, self.detection_radius))
        return config

    def guard_pose_cb(self, data):
        self.guard_pose = data

    def intruder_pose_cb(self, data):
        self.intruder_pose = data

    def get_distance(self, p1, p2):
        return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)

    def get_angle(self, p1, p2):
        return math.atan2(p2.y - p1.y, p2.x - p1.x)

    def find_closest_waypoint_index(self):
        """计算离当前位置最近的巡逻点索引"""
        min_dist = float('inf')
        best_index = 0
        for i, wp in enumerate(self.waypoints):
            temp_pose = Pose()
            temp_pose.x = wp[0]
            temp_pose.y = wp[1]
            dist = self.get_distance(self.guard_pose, temp_pose)
            if dist < min_dist:
                min_dist = dist
                best_index = i
        return best_index

    # 回调函数
    def cb_guard(self, data): self.guard_pose = data
    def cb_t2(self, data): self.t2_pose = data
    def cb_t3(self, data): self.t3_pose = data

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            # 1. 计算到两个入侵者的距离
            dist_t2 = self.get_distance(self.guard_pose, self.t2_pose)
            dist_t3 = self.get_distance(self.guard_pose, self.t3_pose)
            
            # 2. 选取最近的目标
            if dist_t2 < dist_t3:
                target_pose = self.t2_pose
                min_dist = dist_t2
                target_name = "Boss"
            else:
                target_pose = self.t3_pose
                min_dist = dist_t3
                target_name = "Minion"

            self.dist_pub.publish(min_dist)

            # 3. 状态机
            if min_dist < self.detection_radius:
                if self.state != "CHASE":
                    rospy.logwarn(">>> 发现目标 (%s)！追捕开始 <<<" % target_name)
                    self.state = "CHASE"
            else:
                if self.state != "PATROL":
                    rospy.loginfo(">>> 目标丢失，恢复巡逻 <<<")
                    self.current_wp_index = self.find_closest_waypoint_index()
                    self.state = "PATROL"


            # --- 控制逻辑 ---
            cmd = Twist()
            if self.state == "CHASE":
                # 追向选定的最近目标
                target_ang = self.get_angle(self.guard_pose, target_pose)
                angle_error = target_ang - self.guard_pose.theta
                while angle_error > math.pi: angle_error -= 2*math.pi
                while angle_error < -math.pi: angle_error += 2*math.pi
                
                cmd.linear.x = self.kp_linear * min_dist
                cmd.angular.z = self.kp_angular * angle_error
                cmd.linear.x = min(cmd.linear.x, 2.5) # 追捕时允许更快

            elif self.state == "PATROL":
                target_wp = self.waypoints[self.current_wp_index]
                tp = Pose(); tp.x = target_wp[0]; tp.y = target_wp[1]
                d_wp = self.get_distance(self.guard_pose, tp)
                t_ang = self.get_angle(self.guard_pose, tp)
                ang_err = t_ang - self.guard_pose.theta
                while ang_err > math.pi: ang_err -= 2*math.pi
                while ang_err < -math.pi: ang_err += 2*math.pi
                
                cmd.linear.x = 0.8 * d_wp
                cmd.angular.z = 4.0 * ang_err
                cmd.linear.x = min(cmd.linear.x, 1.2)
                
                if d_wp < self.wp_tolerance:
                    self.current_wp_index = (self.current_wp_index + 1) % len(self.waypoints)

            self.pub.publish(cmd)
            rate.sleep()

if __name__ == '__main__':
    try:
        guard = TurtleGuard()
        guard.run()
    except rospy.ROSInterruptException:
        pass
