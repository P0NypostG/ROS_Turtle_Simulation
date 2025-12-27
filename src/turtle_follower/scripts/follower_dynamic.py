#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time
from dynamic_reconfigure.server import Server
from turtle_follower.cfg import FollowerDynamicConfig

class DynamicPIDController:
    """支持动态重配置的PID控制器"""
    def __init__(self, Kp=1.0, Ki=0.0, Kd=0.0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = time.time()
    
    def update(self, error, windup_guard=10.0):
        current_time = time.time()
        dt = current_time - self.prev_time
        
        if dt <= 0:
            dt = 0.01
        
        # P项
        P = self.Kp * error
        
        # I项
        self.integral += error * dt
        if self.integral > windup_guard:
            self.integral = windup_guard
        elif self.integral < -windup_guard:
            self.integral = -windup_guard
        I = self.Ki * self.integral
        
        # D项
        derivative = (error - self.prev_error) / dt
        D = self.Kd * derivative
        
        self.prev_error = error
        self.prev_time = current_time
        
        return P + I + D
    
    def update_gains(self, Kp, Ki, Kd):
        """更新PID增益"""
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        rospy.loginfo(f"PID增益更新: Kp={Kp}, Ki={Ki}, Kd={Kd}")
    
    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = time.time()

class DynamicFollower:
    def __init__(self, robot_name='turtle2'):
        rospy.init_node('dynamic_follower', anonymous=True)
        
        self.robot_name = robot_name
        
        # 创建机器人
        rospy.wait_for_service('spawn')
        from turtlesim.srv import Spawn
        spawner = rospy.ServiceProxy('spawn', Spawn)
        try:
            spawner(5.0, 5.0, 0.0, robot_name)
        except:
            rospy.logwarn(f"{robot_name} 可能已存在")
        
        # 订阅器
        rospy.Subscriber('/turtle1/pose', Pose, self.leader_pose_callback)
        rospy.Subscriber(f'/{robot_name}/pose', Pose, self.follower_pose_callback)
        
        # 发布器
        self.cmd_pub = rospy.Publisher(f'/{robot_name}/cmd_vel', Twist, queue_size=10)
        
        # 位姿存储
        self.leader_pose = Pose()
        self.follower_pose = Pose()
        
        # 默认PID参数
        self.Kp_distance = 1.0
        self.Ki_distance = 0.05
        self.Kd_distance = 0.2
        self.Kp_angle = 4.0
        self.Ki_angle = 0.1
        self.Kd_angle = 0.5
        
        # PID控制器
        self.distance_pid = DynamicPIDController(
            self.Kp_distance, self.Ki_distance, self.Kd_distance)
        self.angle_pid = DynamicPIDController(
            self.Kp_angle, self.Ki_angle, self.Kd_angle)
        
        # 其他参数
        self.target_distance = 1.0
        self.max_linear_speed = 2.0
        self.max_angular_speed = 2.0
        self.update_rate = 20.0
        
        # 动态重配置服务器
        self.dyn_reconf_server = Server(FollowerDynamicConfig, 
                                         self.dynamic_reconfigure_callback)
        
        # 控制循环
        self.rate = rospy.Rate(self.update_rate)
        
        rospy.loginfo("动态PID跟随者启动成功！使用rqt_reconfigure调整参数")
    
    def dynamic_reconfigure_callback(self, config, level):
        """动态重配置回调函数"""
        # 更新PID增益
        if (self.Kp_distance != config.Kp_distance or 
            self.Ki_distance != config.Ki_distance or 
            self.Kd_distance != config.Kd_distance):
            self.distance_pid.update_gains(
                config.Kp_distance, config.Ki_distance, config.Kd_distance)
        
        if (self.Kp_angle != config.Kp_angle or 
            self.Ki_angle != config.Ki_angle or 
            self.Kd_angle != config.Kd_angle):
            self.angle_pid.update_gains(
                config.Kp_angle, config.Ki_angle, config.Kd_angle)
        
        # 更新参数
        self.Kp_distance = config.Kp_distance
        self.Ki_distance = config.Ki_distance
        self.Kd_distance = config.Kd_distance
        self.Kp_angle = config.Kp_angle
        self.Ki_angle = config.Ki_angle
        self.Kd_angle = config.Kd_angle
        self.target_distance = config.target_distance
        self.max_linear_speed = config.max_linear_speed
        self.max_angular_speed = config.max_angular_speed
        
        # 更新控制频率
        if self.update_rate != config.update_rate:
            self.update_rate = config.update_rate
            self.rate = rospy.Rate(self.update_rate)
        
        rospy.loginfo(f"参数已更新: 距离PID=[{config.Kp_distance:.2f}, {config.Ki_distance:.3f}, {config.Kd_distance:.2f}], "
                      f"角度PID=[{config.Kp_angle:.2f}, {config.Ki_angle:.3f}, {config.Kd_angle:.2f}], "
                      f"目标距离={config.target_distance:.2f}")
        
        return config
    
    def leader_pose_callback(self, data):
        self.leader_pose = data
    
    def follower_pose_callback(self, data):
        self.follower_pose = data
    
    def compute_control(self):
        """计算控制指令"""
        dx = self.leader_pose.x - self.follower_pose.x
        dy = self.leader_pose.y - self.follower_pose.y
        
        distance = math.sqrt(dx**2 + dy**2)
        target_angle = math.atan2(dy, dx)
        
        distance_error = distance - self.target_distance
        angle_error = target_angle - self.follower_pose.theta
        
        # 角度归一化
        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi
        
        # 使用PID计算控制量
        linear_vel = self.distance_pid.update(distance_error)
        angular_vel = self.angle_pid.update(angle_error)
        
        # 速度限制
        linear_vel = max(-self.max_linear_speed, 
                        min(self.max_linear_speed, linear_vel))
        angular_vel = max(-self.max_angular_speed, 
                         min(self.max_angular_speed, angular_vel))
        
        cmd = Twist()
        cmd.linear.x = linear_vel
        cmd.angular.z = angular_vel
        
        return cmd, distance_error, angle_error
    
    def run(self):
        """主循环"""
        while not rospy.is_shutdown():
            try:
                cmd, dist_err, ang_err = self.compute_control()
                self.cmd_pub.publish(cmd)
                
                # 显示当前参数和误差
                rospy.loginfo_throttle(2.0,
                    f"动态PID - 距离误差: {dist_err:.3f}, 角度误差: {ang_err:.3f}\n"
                    f"当前参数: Kp_d={self.Kp_distance:.2f}, Ki_d={self.Ki_distance:.3f}, "
                    f"Kd_d={self.Kd_distance:.2f}, 目标距离={self.target_distance:.2f}")
                
            except Exception as e:
                rospy.logwarn(f"错误: {str(e)}")
            
            self.rate.sleep()

if __name__ == '__main__':
    try:
        # 支持命令行参数指定机器人名称
        import sys
        if len(sys.argv) > 1:
            robot_name = sys.argv[1]
            follower = DynamicFollower(robot_name)
        else:
            follower = DynamicFollower()
        
        follower.run()
    except rospy.ROSInterruptException:
        pass
