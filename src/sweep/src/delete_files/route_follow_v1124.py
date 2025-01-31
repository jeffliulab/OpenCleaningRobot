#!/usr/bin/env python3
import rospy
import yaml
import os
import math
from geometry_msgs.msg import Twist, PoseStamped
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String, ColorRGBA
import tf
from tf.transformations import euler_from_quaternion
import numpy as np

class MapData:
    OBSTACLE = -1
    UNVISITED = 0
    VISITED = 1

class RouteFollower:
    def __init__(self):
        rospy.init_node('route_follower', anonymous=True)
        
        # 参数设置
        self.POSITION_TOLERANCE = 0.1   # 到达目标点的位置误差
        self.ANGLE_TOLERANCE = 0.1     # 角度误差
        self.LINEAR_SPEED = 0.2        # 线速度
        self.ANGULAR_SPEED = 0.3       # 角速度
        
        # 数据存储
        self.path_points = []
        self.path_connections = []
        self.current_pose = None
        self.current_path_index = None
        
        # 创建发布器和订阅器
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.status_pub = rospy.Publisher('/route_status', String, queue_size=1)
        # 使用新的话题名称发布状态可视化
        self.visualization_pub = rospy.Publisher('/route_status_visualization', MarkerArray, queue_size=1, latch=True)
        
        # 订阅AMCL位姿
        self.amcl_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_callback)
        
        # TF监听器
        self.tf_listener = tf.TransformListener()
        
        # 加载路径文件
        self.path_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "pathfiles")
        if not os.path.exists(self.path_dir):
            os.makedirs(self.path_dir)
            
        self.load_latest_path()
        rospy.loginfo("Route follower initialized")
        
    def load_latest_path(self):
        """加载最新的路径文件"""
        try:
            path_files = [f for f in os.listdir(self.path_dir) 
                         if f.startswith('coverage_path_') and f.endswith('.yaml')]
            if not path_files:
                rospy.logwarn("No path files found")
                return False
                
            latest_file = sorted(path_files)[-1]
            path_file = os.path.join(self.path_dir, latest_file)
            rospy.loginfo(f"Loading path from {path_file}")
            
            with open(path_file, 'r') as f:
                data = yaml.safe_load(f)
                self.path_points = data['path_points']
                rospy.loginfo(f"Loaded {len(self.path_points)} path points")
            return True
                
        except Exception as e:
            rospy.logerr(f"Error loading path file: {str(e)}")
            return False

    def amcl_callback(self, msg):
        """处理AMCL位姿更新"""
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        # 转换四元数为欧拉角
        _, _, yaw = euler_from_quaternion([
            orientation.x, orientation.y, orientation.z, orientation.w])
        self.current_pose = (position.x, position.y, yaw)

    def get_robot_pose(self):
        """获取机器人当前位姿"""
        if self.current_pose is None:
            rospy.logwarn("Waiting for AMCL pose...")
            return None
        return self.current_pose

    def check_localization_accuracy(self):
        """使用AMCL的协方差来判断定位精度"""
        try:
            # 等待最新的AMCL位姿
            amcl_pose = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped, timeout=1.0)
            
            # 获取协方差矩阵中的位置不确定性
            cov = amcl_pose.pose.covariance
            x_uncertainty = cov[0]
            y_uncertainty = cov[7]
            
            # 设定阈值
            UNCERTAINTY_THRESHOLD = 0.05  # 5cm的不确定性
            
            if max(x_uncertainty, y_uncertainty) > UNCERTAINTY_THRESHOLD:
                return False, "AMCL localization uncertainty too high"
                
            return True, "Localization accuracy sufficient"
            
        except Exception as e:
            return False, f"Error checking AMCL accuracy: {str(e)}"

    def find_valid_connections(self, points):
        """找出所有有效的路径连接"""
        connections = []
        visited = set()
        
        if not points:
            return connections
            
        current_point = points[0]
        visited.add(0)
        
        while len(visited) < len(points):
            best_distance = float('inf')
            best_next_point = None
            best_next_idx = None
            
            for i, point in enumerate(points):
                if i in visited:
                    continue
                    
                distance = math.hypot(
                    current_point['grid_x'] - point['grid_x'],
                    current_point['grid_y'] - point['grid_y']
                )
                
                if distance < best_distance:
                    best_distance = distance
                    best_next_point = point
                    best_next_idx = i
            
            if best_next_point is None:
                break
                
            connections.append((current_point, best_next_point))
            visited.add(best_next_idx)
            current_point = best_next_point
        
        return connections

    def find_initial_point(self, robot_pose):
        """找到最近的未访问的路径起始点"""
        min_distance = float('inf')
        nearest_point = None
        
        # 只考虑作为路径起点的未访问点
        start_points = set(conn[0]['id'] for conn in self.path_connections)
        for point in self.path_points:
            if point['status'] != MapData.UNVISITED:
                continue
            if point['id'] not in start_points:
                continue
                
            dist = math.hypot(
                point['world_x'] - robot_pose[0],
                point['world_y'] - robot_pose[1]
            )
            if dist < min_distance:
                min_distance = dist
                nearest_point = point
                
        return nearest_point

    def get_next_point(self, current_point):
        """根据路径连接关系获取下一个点"""
        for start, end in self.path_connections:
            if start['id'] == current_point['id']:
                return end
        return None

    def publish_point_status(self):
        """发布所有点的状态用于可视化"""
        point_markers = MarkerArray()
        
        for i, point in enumerate(self.path_points):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "path_points"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position.x = point['world_x']
            marker.pose.position.y = point['world_y']
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = marker.scale.y = marker.scale.z = 0.1
            
            # 根据点的状态设置颜色
            if point['status'] == MapData.VISITED:
                marker.color = ColorRGBA(0.0, 1.0, 0.0, 0.8)  # 绿色
            else:
                marker.color = ColorRGBA(1.0, 0.0, 0.0, 0.8)  # 红色
                
            point_markers.markers.append(marker)
        
        self.visualization_pub.publish(point_markers)

    def move_to_point(self, target_point):
        """移动到目标点"""
        if not self.get_robot_pose():
            return False
            
        rate = rospy.Rate(10)  # 10Hz
        cmd_vel = Twist()
        
        start_time = rospy.Time.now()
        timeout = rospy.Duration(30.0)  # 30秒超时
        
        while not rospy.is_shutdown():
            current_pose = self.get_robot_pose()
            if not current_pose:
                continue
                
            # 检查是否超时
            if (rospy.Time.now() - start_time) > timeout:
                rospy.logwarn("Move to point timeout")
                self.stop_robot()
                return False
                
            # 计算距离和角度
            dx = target_point['world_x'] - current_pose[0]
            dy = target_point['world_y'] - current_pose[1]
            distance = math.hypot(dx, dy)
            target_angle = math.atan2(dy, dx)
            
            # 如果到达目标点
            if distance < self.POSITION_TOLERANCE:
                self.stop_robot()
                return True
                
            # 计算角度差
            angle_diff = target_angle - current_pose[2]
            angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
            
            # 根据角度差决定运动方式
            if abs(angle_diff) > self.ANGLE_TOLERANCE:
                # 原地旋转到正确方向
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = self.ANGULAR_SPEED if angle_diff > 0 else -self.ANGULAR_SPEED
            else:
                # 保持方向的同时前进
                cmd_vel.linear.x = min(self.LINEAR_SPEED, distance)
                cmd_vel.angular.z = 0.5 * angle_diff  # 使用比例控制保持方向
                
            self.cmd_vel_pub.publish(cmd_vel)
            rate.sleep()
            
        self.stop_robot()
        return False

    def stop_robot(self):
        """停止机器人"""
        cmd_vel = Twist()
        self.cmd_vel_pub.publish(cmd_vel)

    def update_point_status(self, point_id):
        """更新点的状态并发布可视化"""
        for point in self.path_points:
            if point['id'] == point_id:
                point['status'] = MapData.VISITED
                # 发布可视化更新
                self.publish_point_status()
                return True
        return False

    def follow_route(self):
        """主控制循环"""
        rospy.loginfo("Starting route following...")
        rate = rospy.Rate(10)  # 10Hz
        
        while not rospy.is_shutdown():
            # 1. 检查AMCL定位精度
            accuracy_ok, msg = self.check_localization_accuracy()
            if not accuracy_ok:
                rospy.logwarn(msg)
                rate.sleep()
                continue
            
            # 2. 获取机器人位置
            robot_pose = self.get_robot_pose()
            if not robot_pose:
                continue
                
            # 3. 如果还没有开始跟随路径，找到起始点
            if self.current_path_index is None:
                initial_point = self.find_initial_point(robot_pose)
                if not initial_point:
                    rospy.loginfo("No available starting points")
                    break
                self.current_path_index = initial_point['id']
                
            # 4. 获取当前目标点
            current_target = None
            for point in self.path_points:
                if point['id'] == self.current_path_index:
                    current_target = point
                    break
                    
            if not current_target:
                rospy.loginfo("Path completed!")
                break
                
            # 5. 移动到目标点
            if self.move_to_point(current_target):
                # 更新点的状态
                self.update_point_status(current_target['id'])
                
                # 获取下一个点
                next_point = self.get_next_point(current_target)
                if next_point:
                    self.current_path_index = next_point['id']
                else:
                    rospy.loginfo("Reached end of path")
                    break
            else:
                rospy.logwarn("Failed to reach target point")
            
            rate.sleep()
        
        self.stop_robot()

    def run(self):
        """运行路径跟随器"""
        # 等待初始定位
        rospy.sleep(2)  # 给AMCL一些时间进行初始定位
        
        if not self.path_points:
            rospy.logerr("No path points loaded. Cannot start route following.")
            return
            
        try:
            # 计算路径连接关系
            self.path_connections = self.find_valid_connections(self.path_points)
            # 初始化点的可视化
            self.publish_point_status()
            # 开始跟随路径
            self.follow_route()
        except KeyboardInterrupt:
            self.stop_robot()
            rospy.loginfo("Route following interrupted by user")
        except Exception as e:
            self.stop_robot()
            rospy.logerr(f"Route following error: {str(e)}")

if __name__ == "__main__":
    try:
        follower = RouteFollower()
        follower.run()
    except rospy.ROSInterruptException:
        pass