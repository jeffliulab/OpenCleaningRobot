#!/usr/bin/env python3
import rospy
import yaml
import os
import math
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String, ColorRGBA
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
import numpy as np
from geometry_msgs.msg import Point, PoseStamped


class MapData:
    OBSTACLE = -1    # obstacles points, black
    UNVISITED = 0    # unvisited, red
    VISITED = 1      # visited points, green

class RouteFollower:
    def __init__(self):
        rospy.init_node('route_follower', anonymous=True)
        
        # Modified Settings
        self.POSITION_TOLERANCE = 0.15   # 位置容差
        self.ANGLE_TOLERANCE = 0.1      # angle tolerance (角度误差)
        self.LINEAR_SPEED = 0.2         # linear speed
        self.ANGULAR_SPEED = 0.3        # angular speed
        self.SAFETY_DISTANCE = 0.2      # 安全距离
        self.SAMPLING_INTERVAL = 5      # sampling interval, keep same with route_plan
        self.MAX_REACHABLE_DISTANCE = 5.0  # 最大可达距离
        
        # 定义读取路径
        self.read_path_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "pathfiles")
        
        # Store the data
        self.path_points = []
        self.path_connections = []      # store connections
        self.current_pose = None
        self.current_path_index = None
        self.obstacle_points = []
        self.current_path = Path()      # curent path
        
        # Pub and Sub
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.status_pub = rospy.Publisher('/route_status', String, queue_size=1)
        self.visualization_pub = rospy.Publisher('/route_status_visualization', MarkerArray, queue_size=1, latch=True)
        self.path_pub = rospy.Publisher('/current_path', Path, queue_size=1, latch=True)
        
        # AMCL and LiDAR
        self.amcl_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_callback)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        
        self.load_latest_path()
        rospy.loginfo("Route follower initialized")

    def load_latest_path(self):
        """Load the latest path file and initialize all points as unvisited"""
        try:
            rospy.loginfo("Loading latest path file...")
            path_files = [f for f in os.listdir(self.read_path_dir) 
                        if f.startswith('coverage_path_') and f.endswith('.yaml')]
            if not path_files:
                rospy.logwarn("No path files found")
                return False
                    
            latest_file = sorted(path_files)[-1]
            path_file = os.path.join(self.read_path_dir, latest_file)
            rospy.loginfo(f"Loading path from {path_file}")
            
            with open(path_file, 'r') as f:
                data = yaml.safe_load(f)
                self.path_points = data['path_points']
                
            # 初始化所有点为未访问状态
            for point in self.path_points:
                point['status'] = MapData.UNVISITED
                
            self.build_path_connections()
            rospy.loginfo(f"Loaded {len(self.path_points)} path points")
            return True
                
        except Exception as e:
            rospy.logerr(f"Error loading path file: {str(e)}")
            return False
        
    def build_path_connections(self):
        """Build path connection relationship, consistent with route_plan"""
        rospy.loginfo("Building path connections...")
        self.path_connections = []
        visited = set()
        
        if not self.path_points:
            rospy.logwarn("No path points available for building connections")
            return
            
        current_point = self.path_points[0]
        visited.add(0)
        
        while len(visited) < len(self.path_points):
            best_distance = float('inf')
            best_next_point = None
            best_next_idx = None
            
            # find nearest unvisited point
            for i, point in enumerate(self.path_points):
                if i in visited:
                    continue
                
                # use sampling interval to find the distance
                dx = abs(current_point['grid_x'] - point['grid_x'])
                dy = abs(current_point['grid_y'] - point['grid_y'])
                
                if not ((dx == self.SAMPLING_INTERVAL and dy == 0) or 
                        (dx == 0 and dy == self.SAMPLING_INTERVAL)):
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
                # If no adjacent point is found, choose a new starting point
                unvisited = set(range(len(self.path_points))) - visited
                if unvisited:
                    next_start_idx = min(unvisited)
                    current_point = self.path_points[next_start_idx]
                    visited.add(next_start_idx)
                    continue
                else:
                    break
            
            self.path_connections.append((current_point['id'], best_next_point['id']))
            visited.add(best_next_idx)
            current_point = best_next_point
        
        rospy.loginfo(f"Built {len(self.path_connections)} path connections")

    def is_point_reachable(self, point):
        """检查点是否在可达范围内"""
        if not self.current_pose:
            return False
        distance = math.hypot(
            point['world_x'] - self.current_pose[0],
            point['world_y'] - self.current_pose[1]
        )
        rospy.loginfo(f"Distance to point {point['id']}: {distance}")
        return distance < self.MAX_REACHABLE_DISTANCE

    def get_next_planned_point(self, current_point_id):
        """Get the next point according to the planned path"""
        rospy.loginfo(f"Getting next planned point from point {current_point_id}")
        # Find the next point in a route connection
        for start_id, end_id in self.path_connections:
            if start_id == current_point_id:
                # Find the complete information of the target point
                for point in self.path_points:
                    if point['id'] == end_id and point['status'] == MapData.UNVISITED:
                        # Make sure the next point is accessible
                        if self.check_path_safety(point) and self.is_point_reachable(point):
                            rospy.loginfo(f"Found next point: {point}")
                            return point
                        else:
                            rospy.logwarn(f"Point {point['id']} is not safely reachable")
        rospy.loginfo("No next planned point found")
        return None

    def find_nearest_accessible_point(self):
        """Find the nearest accessible unvisited point"""
        if not self.current_pose:
            rospy.logwarn("No current pose available")
            return None
        
        rospy.loginfo(f"Finding nearest point from position: {self.current_pose}")
        points_checked = 0
        points_failed_safety = 0
        points_failed_distance = 0
        
        min_distance = float('inf')
        nearest_point = None
        
        for point in self.path_points:
            points_checked += 1
            if point['status'] != MapData.UNVISITED:
                continue
                
            distance = math.hypot(
                point['world_x'] - self.current_pose[0],
                point['world_y'] - self.current_pose[1]
            )
            
            if distance > self.MAX_REACHABLE_DISTANCE:
                points_failed_distance += 1
                continue
                
            if distance < min_distance:
                if self.check_path_safety(point):
                    min_distance = distance
                    nearest_point = point
                else:
                    points_failed_safety += 1
        
        rospy.loginfo(f"Checked {points_checked} points")
        rospy.loginfo(f"Failed safety check: {points_failed_safety}")
        rospy.loginfo(f"Failed distance check: {points_failed_distance}")
        rospy.loginfo(f"Nearest point found: {nearest_point}")
        return nearest_point

    def amcl_callback(self, msg):
        """AMCL callback"""
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([
            orientation.x, orientation.y, orientation.z, orientation.w])
        self.current_pose = (position.x, position.y, yaw)
        rospy.logdebug(f"Current pose updated: {self.current_pose}")

    def laser_callback(self, scan_msg):
        """LiDAR callback"""
        if not self.current_pose:
            return

        self.obstacle_points = []
        angle = scan_msg.angle_min
        
        for r in scan_msg.ranges:
            # 过滤掉无效的0值和最大值
            if 0.1 < r < scan_msg.range_max:  # 添加最小阈值0.1
                x = self.current_pose[0] + r * math.cos(angle + self.current_pose[2])
                y = self.current_pose[1] + r * math.sin(angle + self.current_pose[2])
                self.obstacle_points.append((x, y))
            angle += scan_msg.angle_increment

    def check_path_safety(self, target_point):
        """Check if the path to the destination is safe"""
        if not self.current_pose or not self.obstacle_points:
            rospy.loginfo("No current pose or obstacle points available")
            return True
                
        path_start = np.array([self.current_pose[0], self.current_pose[1]])
        path_end = np.array([target_point['world_x'], target_point['world_y']])
        path_vector = path_end - path_start
        path_length = np.linalg.norm(path_vector)
        
        if path_length == 0:
            return True

        # 记录危险点数量    
        danger_points = 0
                
        for obs_point in self.obstacle_points:
            obs_vector = np.array(obs_point) - path_start
            # 计算投影
            projection = np.dot(obs_vector, path_vector) / path_length
            
            if 0 <= projection <= path_length:
                # 计算垂直距离
                distance = abs(np.cross(path_vector, obs_vector)) / path_length
                if distance < self.SAFETY_DISTANCE:
                    danger_points += 1
                    if danger_points > 5:  # 允许路径上有少量障碍点
                        rospy.loginfo(f"Path blocked: {danger_points} danger points detected")
                        return False

        return True

    def publish_point_status(self):
        """Publish the status of all points for visualization"""
        point_markers = MarkerArray()
        
        # 发布点标记
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
            
            if point['status'] == MapData.VISITED:
                marker.color = ColorRGBA(0.0, 1.0, 0.0, 0.8)  # green
            elif point['status'] == MapData.OBSTACLE:
                marker.color = ColorRGBA(0.0, 0.0, 0.0, 0.8)  # black
            else:
                marker.color = ColorRGBA(1.0, 0.0, 0.0, 0.8)  # red
                
            point_markers.markers.append(marker)

        # 发布连接标记
        for i, (start_id, end_id) in enumerate(self.path_connections):
            start_point = next(p for p in self.path_points if p['id'] == start_id)
            end_point = next(p for p in self.path_points if p['id'] == end_id)
            
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "path_connections"
            marker.id = i + len(self.path_points)  # 避免ID冲突
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            
            # Set the arrow start and end points
            marker.points = [
                Point(x=start_point['world_x'], y=start_point['world_y'], z=0),
                Point(x=end_point['world_x'], y=end_point['world_y'], z=0)
            ]
            
            # Set the arrow size
            marker.scale.x = 0.02  # Arrow width
            marker.scale.y = 0.04  # Arrow head width
            marker.scale.z = 0.01  # Arrow height
            
            # Arrow color
            marker.color = ColorRGBA(0.3, 0.3, 1.0, 0.8) # blue
            
            point_markers.markers.append(marker)
        
        self.visualization_pub.publish(point_markers)
        rospy.loginfo("Published point status and path connections")

    def publish_current_path(self, target_point):
        """Publish the current path for visualization"""
        if not self.current_pose:
            rospy.logwarn("Cannot publish path: no current pose")
            return
            
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = rospy.Time.now()

        # Add starting point (current location)
        start_pose = PoseStamped()
        start_pose.header = path_msg.header
        start_pose.pose.position.x = self.current_pose[0]
        start_pose.pose.position.y = self.current_pose[1]
        start_pose.pose.orientation.w = 1.0
        path_msg.poses.append(start_pose)

        # Add Target Point
        target_pose = PoseStamped()
        target_pose.header = path_msg.header
        target_pose.pose.position.x = target_point['world_x']
        target_pose.pose.position.y = target_point['world_y']
        target_pose.pose.orientation.w = 1.0
        path_msg.poses.append(target_pose)

        self.path_pub.publish(path_msg)
        rospy.loginfo(f"Published path to target point {target_point['id']}")

    def move_to_point(self, target_point):
        """Move to target point"""
        if not self.current_pose:
            rospy.logwarn("Cannot move: no current pose")
            return False
            
        rate = rospy.Rate(10)  # 10Hz
        cmd_vel = Twist()
        
        start_time = rospy.Time.now()
        timeout = rospy.Duration(30.0)  # 30 seconds timeout
        
        while not rospy.is_shutdown():
            if not self.current_pose:
                continue
                
            # Checking for timeout
            if (rospy.Time.now() - start_time) > timeout:
                rospy.logwarn("Move to point timeout")
                self.stop_robot()
                return False
                
            # Check path safety
            if not self.check_path_safety(target_point):
                rospy.logwarn("Path blocked by obstacle")
                self.stop_robot()
                return False
                
            # Calculating distances and angles
            dx = target_point['world_x'] - self.current_pose[0]
            dy = target_point['world_y'] - self.current_pose[1]
            distance = math.hypot(dx, dy)
            target_angle = math.atan2(dy, dx)
            
            rospy.logdebug(f"Distance to target: {distance}, Target angle: {target_angle}")
            
            # If reached destination
            if distance < self.POSITION_TOLERANCE:
                self.stop_robot()
                rospy.loginfo(f"Reached target point {target_point['id']}")
                return True
                
            # Calculate the angle difference
            angle_diff = target_angle - self.current_pose[2]
            angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
            
            # Determine the movement mode based on the angle difference
            if abs(angle_diff) > self.ANGLE_TOLERANCE:
                # Rotate to the correct direction
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = self.ANGULAR_SPEED if angle_diff > 0 else -self.ANGULAR_SPEED
                rospy.logdebug("Rotating to align with target")
            else:
                # Keep moving forward while maintaining direction
                cmd_vel.linear.x = min(self.LINEAR_SPEED, distance)
                cmd_vel.angular.z = 0.5 * angle_diff  # Maintain direction using proportional control
                rospy.logdebug("Moving forward to target")
                
            self.cmd_vel_pub.publish(cmd_vel)
            rate.sleep()
            
        self.stop_robot()
        return False

    def stop_robot(self):
        """Stop Robot"""
        cmd_vel = Twist()
        self.cmd_vel_pub.publish(cmd_vel)
        rospy.loginfo("Robot stopped")

    def check_localization_accuracy(self):
        """Check AMCL positioning accuracy"""
        if not self.current_pose:
            return False, "No pose data"
        return True, "Localization accuracy sufficient"

    def update_point_status(self, point_id, new_status=MapData.VISITED):
        """Update point status in memory"""
        for point in self.path_points:
            if point['id'] == point_id:
                point['status'] = new_status
                self.publish_point_status()
                rospy.loginfo(f"Updated point {point_id} status to {new_status}")
                return True
        rospy.logwarn(f"Point {point_id} not found")
        return False

    def follow_route(self):
        """Main control loop"""
        rospy.loginfo("Starting route following...")
        rate = rospy.Rate(10)  # 10Hz
        
        # 确保初始化完成
        rospy.sleep(2.0)  # 等待AMCL初始化
        
        consecutive_failures = 0  # 添加连续失败计数器
        MAX_FAILURES = 1  # 最大连续失败次数
        
        while not rospy.is_shutdown():
            # Check positioning
            accuracy_ok, msg = self.check_localization_accuracy()
            if not accuracy_ok:
                rospy.logwarn(msg)
                rate.sleep()
                continue
                
            # 如果没有当前目标点，寻找新的目标点
            if self.current_path_index is None:
                rospy.loginfo("Looking for next target point...")
                # 首先寻找最近的可达点作为新的起点
                next_point = self.find_nearest_accessible_point()
                
                if not next_point:
                    if consecutive_failures >= MAX_FAILURES:
                        rospy.loginfo("Maximum consecutive failures reached. Mission complete!")
                        break
                    consecutive_failures += 1
                    rate.sleep()
                    continue
                    
                consecutive_failures = 0
                self.current_path_index = next_point['id']
                rospy.loginfo(f"Found nearest point as new starting point: {self.current_path_index}")
            
            # 获取当前目标点信息
            current_target = None
            for point in self.path_points:
                if point['id'] == self.current_path_index:
                    current_target = point
                    break
            
            if not current_target:
                rospy.logwarn("Current target point not found")
                self.current_path_index = None
                continue
            
            # 检查路径安全性
            if not self.check_path_safety(current_target):
                rospy.logwarn("Path to target blocked")
                self.current_path_index = None
                consecutive_failures += 1
                continue
            
            # 发布当前路径
            self.publish_current_path(current_target)
            
            # 移动到目标点
            if self.move_to_point(current_target):
                # 成功到达目标点
                self.update_point_status(current_target['id'], MapData.VISITED)
                consecutive_failures = 0
                
                # 尝试获取路径规划中的下一个点
                next_planned_point = self.get_next_planned_point(current_target['id'])
                
                if next_planned_point:
                    # 如果有计划路径上的下一个点，继续沿着路径执行
                    self.current_path_index = next_planned_point['id']
                    rospy.loginfo(f"Following planned path to point: {self.current_path_index}")
                else:
                    # 如果没有计划路径上的下一个点，重置current_path_index以寻找新的最近点
                    self.current_path_index = None
                    rospy.loginfo("Current path segment completed, will find new nearest point")
            else:
                rospy.logwarn("Failed to reach target point")
                consecutive_failures += 1
                if consecutive_failures >= MAX_FAILURES:
                    self.update_point_status(current_target['id'], MapData.OBSTACLE)
                self.current_path_index = None
            
            rate.sleep()
        
        self.stop_robot()
        rospy.loginfo("Route following completed")


    def run(self):
        """Run Path Follower"""
        # Waiting for initial positioning
        rospy.sleep(2)  # Give AMCL some time for initial positioning
        
        if not self.path_points:
            rospy.logerr("No path points loaded. Cannot start route following.")
            return
            
        try:
            self.follow_route()
        except KeyboardInterrupt:
            self.stop_robot()
            rospy.loginfo("Route following interrupted by user")
        except Exception as e:
            self.stop_robot()
            rospy.logerr(f"Route following error: {str(e)}")
            raise

if __name__ == "__main__":
    try:
        follower = RouteFollower()
        follower.run()
    except rospy.ROSInterruptException:
        pass