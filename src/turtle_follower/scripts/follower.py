#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time

class PIDController:
    """PID控制器类"""
    def __init__(self, Kp, Ki, Kd, windup_guard=10.0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.windup_guard = windup_guard
        
        # 状态变量
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = time.time()
    
    def update(self, error):
        """更新PID控制器并返回控制量"""
        current_time = time.time()
        dt = current_time - self.prev_time
        
        # 防止除零
        if dt <= 0:
            dt = 0.01
        
        # 比例项
        P = self.Kp * error
        
        # 积分项（带抗饱和）
        self.integral += error * dt
        # 积分限幅
        if self.integral > self.windup_guard:
            self.integral = self.windup_guard
        elif self.integral < -self.windup_guard:
            self.integral = -self.windup_guard
        I = self.Ki * self.integral
        
        # 微分项
        derivative = (error - self.prev_error) / dt
        D = self.Kd * derivative
        
        # 更新状态
        self.prev_error = error
        self.prev_time = current_time
        
        return P + I + D
    
    def reset(self):
        """重置控制器状态"""
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = time.time()

class FollowerTurtle:
    def __init__(self, robot_name='turtle2', start_x=5.0, start_y=5.0):
        rospy.init_node('follower_turtle', anonymous=True)
        
        self.robot_name = robot_name
        self.cmd_topic = f'/{robot_name}/cmd_vel'
        self.pose_topic = f'/{robot_name}/pose'
        
        # 创建机器人（如果不存在）
        rospy.wait_for_service('spawn')
        from turtlesim.srv import Spawn
        spawner = rospy.ServiceProxy('spawn', Spawn)
        try:
            spawner(start_x, start_y, 0.0, robot_name)
            rospy.loginfo(f"已生成机器人: {robot_name}")
        except Exception as e:
            rospy.logwarn(f"{robot_name} 可能已存在: {str(e)}")
        
        # 订阅器：获取 leader 和 follower 位姿
        rospy.Subscriber('/turtle1/pose', Pose, self.leader_pose_callback)
        rospy.Subscriber(self.pose_topic, Pose, self.follower_pose_callback)
        
        # 发布器：控制 follower 运动
        self.cmd_pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=10)
        
        # 存储位姿数据
        self.leader_pose = Pose()
        self.follower_pose = Pose()
        
        # PID控制器：距离控制
        self.distance_pid = PIDController(
            Kp=1.0,    # 比例系数
            Ki=0.05,   # 积分系数
            Kd=0.2,    # 微分系数
            windup_guard=5.0
        )
        
        # PID控制器：角度控制
        self.angle_pid = PIDController(
            Kp=4.0,    # 比例系数
            Ki=0.1,    # 积分系数
            Kd=0.5,    # 微分系数
            windup_guard=3.0
        )
        
        self.target_distance = 1.0  # 目标跟随距离
        
        # 主循环频率
        self.rate = rospy.Rate(20)  # 20Hz
        
        rospy.loginfo(f"PID Follower {robot_name} 启动成功！")
    
    def leader_pose_callback(self, data):
        """回调函数：更新 leader 位姿"""
        self.leader_pose = data
    
    def follower_pose_callback(self, data):
        """回调函数：更新 follower 位姿"""
        self.follower_pose = data
    
    def compute_control(self):
        """使用PID计算控制指令"""
        # 计算相对位置
        dx = self.leader_pose.x - self.follower_pose.x
        dy = self.leader_pose.y - self.follower_pose.y
        
        # 计算距离和角度
        distance = math.sqrt(dx**2 + dy**2)
        target_angle = math.atan2(dy, dx)
        
        # 计算误差
        distance_error = distance - self.target_distance
        angle_error = target_angle - self.follower_pose.theta
        
        # 角度归一化到[-pi, pi]
        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi
        
        # 使用PID控制器计算控制量
        linear_vel = self.distance_pid.update(distance_error)
        angular_vel = self.angle_pid.update(angle_error)
        
        # 控制指令
        cmd = Twist()
        cmd.linear.x = linear_vel
        cmd.angular.z = angular_vel
        
        # 限制最大速度
        cmd.linear.x = max(-2.0, min(2.0, cmd.linear.x))
        cmd.angular.z = max(-2.0, min(2.0, cmd.angular.z))
        
        return cmd, distance_error, angle_error
    
    def run(self):
        """主循环"""
        while not rospy.is_shutdown():
            try:
                # 计算并发布控制指令
                cmd, dist_err, ang_err = self.compute_control()
                self.cmd_pub.publish(cmd)
                
                # 可视化调试信息（每0.5秒打印一次）
                rospy.loginfo_throttle(0.5, 
                    f"PID {self.robot_name} - 距离误差: {dist_err:.3f}, 角度误差: {ang_err:.3f}, "
                    f"速度: [{cmd.linear.x:.2f}, {cmd.angular.z:.2f}], "
                    f"积分项: [{self.distance_pid.integral:.3f}, {self.angle_pid.integral:.3f}]")
                
            except Exception as e:
                rospy.logwarn(f"计算控制指令时出错: {str(e)}")
            
            self.rate.sleep()

if __name__ == '__main__':
    try:
        # 支持命令行参数指定机器人名称和初始位置
        import sys
        if len(sys.argv) > 1:
            robot_name = sys.argv[1]
            start_x = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0
            start_y = float(sys.argv[3]) if len(sys.argv) > 3 else 5.0
            follower = FollowerTurtle(robot_name, start_x, start_y)
        else:
            follower = FollowerTurtle()
        follower.run()
    except rospy.ROSInterruptException:
        pass
