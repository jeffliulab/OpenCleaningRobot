#!/usr/bin/env python3
import rospy
import yaml
import os
import math
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import tf
from tf.transformations import euler_from_quaternion

class RouteFollower:
    def __init__(self):
        rospy.init_node('route_follower', anonymous=True)
        
        # 参数设置
        self.linear_speed = 0.2  # 线速度
        self.angular_speed = 0.5  # 角速度
        self.position_tolerance = 0.1  # 位置容差
        self.angle_tolerance = 0.1  # 角度容差
        
        # 路径相关
        self.path_dir = os.path.join(os.path.dirname(__file__), "pathfiles")
        self.current_path = []
        self.current_pose_index = 0
        
        # 机器人当前位置
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        
        # PID控制器参数
        self.kp_linear = 0.5
        self.kp_angular = 1.0
        self.ki_angular = 0.0
        self.kd_angular = 0.0
        self.angular_error_sum = 0.0
        self.last_angular_error = 0.0
        
        # 发布器和订阅器
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pose_sub = rospy.Subscriber('/amcl_pose', PoseStamped, self.pose_callback)
        
        # 加载路径
        self.load_latest_path()
        
        # 控制循环
        self.control_timer = rospy.Timer(rospy.Duration(0.1), self.control_loop)

    def load_latest_path(self):
        """加载最新的路径文件"""
        try:
            path_files = [f for f in os.listdir(self.path_dir) if f.endswith('.yaml')]
            if not path_files:
                rospy.logwarn("No path files found")
                return
                
            latest_file = max(path_files, key=lambda f: os.path.getctime(os.path.join(self.path_dir, f)))
            path_file = os.path.join(self.path_dir, latest_file)
            
            with open(path_file, 'r') as f:
                path_data = yaml.safe_load(f)
                self.current_path = path_data['path_points']
                rospy.loginfo(f"Loaded path from {path_file}")
                
        except Exception as e:
            rospy.logerr(f"Error loading path file: {e}")

    def pose_callback(self, msg):
        """处理位置更新"""
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        
        # 从四元数获取偏航角
        orientation = msg.pose.orientation
        _, _, self.current_theta = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w])

    def get_target_point(self):
        """获取当前目标点"""
        if self.current_pose_index >= len(self.current_path):
            return None
        return self.current_path[self.current_pose_index]

    def calculate_control(self, target_point):
        """计算控制命令"""
        if target_point is None:
            return 0.0, 0.0
            
        # 计算目标点相对位置
        dx = target_point[0] - self.current_x
        dy = target_point[1] - self.current_y
        
        # 计算目标角度
        target_theta = math.atan2(dy, dx)
        
        # 计算角度误差
        angle_error = target_theta - self.current_theta
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
        
        # PID控制器计算角速度
        self.angular_error_sum += angle_error
        angular_error_diff = angle_error - self.last_angular_error
        angular_velocity = (self.kp_angular * angle_error + 
                          self.ki_angular * self.angular_error_sum +
                          self.kd_angular * angular_error_diff)
        
        self.last_angular_error = angle_error
        
        # 计算距离
        distance = math.sqrt(dx*dx + dy*dy)
        
        # 如果角度误差较大，降低线速度
        linear_velocity = self.linear_speed
        if abs(angle_error) > 0.5:
            linear_velocity *= 0.5
            
        return linear_velocity, angular_velocity

    def check_point_reached(self, target_point):
        """检查是否到达目标点"""
        if target_point is None:
            return False
            
        dx = target_point[0] - self.current_x
        dy = target_point[1] - self.current_y
        distance = math.sqrt(dx*dx + dy*dy)
        
        return distance < self.position_tolerance

    def control_loop(self, event):
        """控制循环"""
        if not self.current_path:
            return
            
        target_point = self.get_target_point()
        if target_point is None:
            rospy.loginfo("Path completed!")
            return
            
        # 如果到达当前点，移动到下一个点
        if self.check_point_reached(target_point):
            self.current_pose_index += 1
            rospy.loginfo(f"Reached point {self.current_pose_index-1}")
            return
            
        # 计算和发布控制命令
        linear_vel, angular_vel = self.calculate_control(target_point)
        
        cmd_vel = Twist()
        cmd_vel.linear.x = linear_vel
        cmd_vel.angular.z = angular_vel
        self.cmd_vel_pub.publish(cmd_vel)

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        follower = RouteFollower()
        follower.run()
    except rospy.ROSInterruptException:
        pass