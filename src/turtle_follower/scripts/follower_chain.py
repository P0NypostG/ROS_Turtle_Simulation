#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import sys

class ChainFollower:
    """链式跟随者，可以跟随任意指定的机器人"""
    def __init__(self, follower_name='turtle2', target_name='turtle1', 
                 start_x=5.0, start_y=5.0, follow_distance=1.0):
        rospy.init_node(f'{follower_name}_follower', anonymous=True)
        
        self.follower_name = follower_name
        self.target_name = target_name
        self.follow_distance = follow_distance
        
        # 话题名称
        self.cmd_topic = f'/{follower_name}/cmd_vel'
        self.pose_topic = f'/{follower_name}/pose'
        self.target_pose_topic = f'/{target_name}/pose'
        
        # 创建机器人（如果不存在）
        rospy.wait_for_service('spawn')
        from turtlesim.srv import Spawn
        spawner = rospy.ServiceProxy('spawn', Spawn)
        try:
            spawner(start_x, start_y, 0.0, follower_name)
            rospy.loginfo(f"已生成机器人: {follower_name}")
        except Exception as e:
            rospy.logwarn(f"{follower_name} 可能已存在: {str(e)}")
        
        # 订阅器：获取目标和自身的位姿
        rospy.Subscriber(self.target_pose_topic, Pose, self.target_pose_callback)
        rospy.Subscriber(self.pose_topic, Pose, self.follower_pose_callback)
        
        # 发布器：控制自身运动
        self.cmd_pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=10)
        
        # 存储位姿数据
        self.target_pose = Pose()
        self.follower_pose = Pose()
        
        # 控制参数
        self.Kp_linear = 1.0
        self.Kp_angular = 4.0
        
        # 主循环频率
        self.rate = rospy.Rate(20)
        
        rospy.loginfo(f"{follower_name} 正在跟随 {target_name}，距离: {follow_distance}")
    
    def target_pose_callback(self, data):
        self.target_pose = data
    
    def follower_pose_callback(self, data):
        self.follower_pose = data
    
    def compute_control(self):
        """计算控制指令"""
        # 计算相对位置
        dx = self.target_pose.x - self.follower_pose.x
        dy = self.target_pose.y - self.follower_pose.y
        
        # 计算距离和角度
        distance = math.sqrt(dx**2 + dy**2)
        target_angle = math.atan2(dy, dx)
        
        # 计算误差
        distance_error = distance - self.follow_distance
        angle_error = target_angle - self.follower_pose.theta
        
        # 角度归一化
        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi
        
        # 控制律
        cmd = Twist()
        cmd.linear.x = self.Kp_linear * distance_error
        cmd.angular.z = self.Kp_angular * angle_error
        
        # 限制最大速度
        cmd.linear.x = max(-2.0, min(2.0, cmd.linear.x))
        cmd.angular.z = max(-2.0, min(2.0, cmd.angular.z))
        
        return cmd, distance_error, angle_error
    
    def run(self):
        """主循环"""
        while not rospy.is_shutdown():
            try:
                cmd, dist_err, ang_err = self.compute_control()
                self.cmd_pub.publish(cmd)
                
                rospy.loginfo_throttle(1.0,
                    f"{self.follower_name}→{self.target_name}: "
                    f"距离误差={dist_err:.3f}, 角度误差={ang_err:.3f}")
            except Exception as e:
                rospy.logwarn(f"错误: {str(e)}")
            
            self.rate.sleep()

if __name__ == '__main__':
    try:
        # 从命令行参数获取配置
        if len(sys.argv) >= 3:
            follower_name = sys.argv[1]
            target_name = sys.argv[2]
            start_x = float(sys.argv[3]) if len(sys.argv) > 3 else 5.0
            start_y = float(sys.argv[4]) if len(sys.argv) > 4 else 5.0
            follow_distance = float(sys.argv[5]) if len(sys.argv) > 5 else 1.0
            
            follower = ChainFollower(follower_name, target_name, 
                                    start_x, start_y, follow_distance)
        else:
            rospy.logerr("用法: follower_chain.py <follower_name> <target_name> [start_x] [start_y] [distance]")
            sys.exit(1)
        
        follower.run()
    except rospy.ROSInterruptException:
        pass
