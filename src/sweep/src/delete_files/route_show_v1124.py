#!/usr/bin/env python3
import rospy
import yaml
import os
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

class MapData:
    OBSTACLE = -1    # 障碍物点
    UNVISITED = 0    # 未访问的路径点
    VISITED = 1      # 已访问的路径点

class PointDisplay:
    OBSTACLE_COLOR = ColorRGBA(0.0, 0.0, 0.0, 0.8)    # 黑色
    UNVISITED_COLOR = ColorRGBA(1.0, 0.0, 0.0, 0.8)   # 红色
    VISITED_COLOR = ColorRGBA(0.0, 1.0, 0.0, 0.8)     # 绿色
    POINT_SIZE = 0.1  # 点的大小

class RouteVisualizer:
    def __init__(self):
        rospy.init_node('route_visualizer', anonymous=True)
        
        # 数据存储
        self.map_data = None
        self.path_points = []
        self.current_point_id = 0  # 当前访问点的ID
        
        # 发布器（使用latch=True确保消息持续存在）
        self.points_pub = rospy.Publisher('/path_visualization', MarkerArray, queue_size=1, latch=True)
        self.connection_pub = rospy.Publisher('/path_connections', MarkerArray, queue_size=1, latch=True)
        
        # 加载路径文件
        self.path_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "pathfiles")
        if not os.path.exists(self.path_dir):
            os.makedirs(self.path_dir)
        self.load_latest_path()
        
        # 定时发布
        self.timer = rospy.Timer(rospy.Duration(1.0), self.publish_visualizations)
        rospy.loginfo("Route visualizer initialized")

    def load_latest_path(self):
        """加载最新的路径文件"""
        try:
            path_files = [f for f in os.listdir(self.path_dir) 
                         if f.startswith('coverage_path_') and f.endswith('.yaml')]
            if not path_files:
                rospy.logwarn("No path files found")
                return
                
            latest_file = sorted(path_files)[-1]
            path_file = os.path.join(self.path_dir, latest_file)
            rospy.loginfo(f"Loading path from {path_file}")
            
            with open(path_file, 'r') as f:
                data = yaml.safe_load(f)
                self.map_data = data['map_data']
                self.path_points = data['path_points']
                rospy.loginfo(f"Loaded {len(self.path_points)} path points")
                
        except Exception as e:
            rospy.logerr(f"Error loading path file: {str(e)}")

    def are_adjacent(self, point1, point2):
        """判断两个点是否相邻"""
        sampling_interval = self.path_points[0].get('sampling_interval', 5)  # 默认值5
        dx = abs(point1['grid_x'] - point2['grid_x'])
        dy = abs(point1['grid_y'] - point2['grid_y'])
        return (dx == sampling_interval and dy == 0) or (dx == 0 and dy == sampling_interval)

    def find_valid_connections(self, points):
        """找出所有有效的连接"""
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
            
            # 找到最近的未访问点
            for i, point in enumerate(points):
                if i in visited:
                    continue
                    
                if not self.are_adjacent(current_point, point):
                    continue
                
                distance = ((current_point['grid_x'] - point['grid_x']) ** 2 + 
                          (current_point['grid_y'] - point['grid_y']) ** 2) ** 0.5
                          
                if distance < best_distance:
                    best_distance = distance
                    best_next_point = point
                    best_next_idx = i
            
            if best_next_point is None:
                # 如果找不到下一个点，从剩余未访问点中选择一个新的起点
                unvisited = set(range(len(points))) - visited
                if unvisited:
                    next_start_idx = min(unvisited)
                    current_point = points[next_start_idx]
                    visited.add(next_start_idx)
                    continue
                else:
                    break
            
            connections.append((current_point, best_next_point))
            visited.add(best_next_idx)
            current_point = best_next_point
        
        return connections

    def publish_visualizations(self, event):
        """发布可视化信息"""
        if not self.map_data or not self.path_points:
            return

        # 创建点标记
        point_markers = MarkerArray()
        
        # 添加路径点
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
            
            marker.scale.x = marker.scale.y = marker.scale.z = PointDisplay.POINT_SIZE
            
            # 根据点的状态设置颜色
            if point.get('status', 0) == MapData.VISITED:
                marker.color = PointDisplay.VISITED_COLOR
            else:
                marker.color = PointDisplay.UNVISITED_COLOR
            
            point_markers.markers.append(marker)
        
        # 添加障碍物点
        obstacle_id = len(self.path_points)
        map_array = self.map_data['array']
        resolution = self.map_data['resolution']
        origin_x = self.map_data['origin_x']
        origin_y = self.map_data['origin_y']
        
        # 采样显示障碍物点（每隔几个点显示一个）
        sampling = 5  # 可以调整这个值来改变障碍物点的密度
        for y in range(0, len(map_array), sampling):
            for x in range(0, len(map_array[0]), sampling):
                if map_array[y][x] == MapData.OBSTACLE:
                    marker = Marker()
                    marker.header.frame_id = "map"
                    marker.header.stamp = rospy.Time.now()
                    marker.ns = "obstacle_points"
                    marker.id = obstacle_id
                    marker.type = Marker.SPHERE
                    marker.action = Marker.ADD
                    
                    marker.pose.position.x = x * resolution + origin_x
                    marker.pose.position.y = y * resolution + origin_y
                    marker.pose.position.z = 0.0
                    marker.pose.orientation.w = 1.0
                    
                    marker.scale.x = marker.scale.y = marker.scale.z = PointDisplay.POINT_SIZE
                    marker.color = PointDisplay.OBSTACLE_COLOR
                    
                    point_markers.markers.append(marker)
                    obstacle_id += 1

        # 发布点标记
        if point_markers.markers:
            self.points_pub.publish(point_markers)
            rospy.loginfo_throttle(10, f"Published {len(point_markers.markers)} markers")

        # 创建并发布连接线标记
        connections = self.find_valid_connections(self.path_points)
        connection_markers = MarkerArray()
        
        for i, (start_point, end_point) in enumerate(connections):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "connections"
            marker.id = i
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            
            # 设置起点和终点
            start = Point()
            start.x = start_point['world_x']
            start.y = start_point['world_y']
            start.z = 0.0
            
            end = Point()
            end.x = end_point['world_x']
            end.y = end_point['world_y']
            end.z = 0.0
            
            marker.points = [start, end]
            marker.scale.x = 0.02  # 箭身宽度
            marker.scale.y = 0.04  # 箭头宽度
            marker.color = ColorRGBA(0.3, 0.3, 1.0, 0.8)  # 蓝色
            
            connection_markers.markers.append(marker)
        
        if connection_markers.markers:
            self.connection_pub.publish(connection_markers)
            rospy.loginfo_throttle(10, f"Published {len(connection_markers.markers)} connections")

    def run(self):
        rospy.loginfo("Route visualizer running...")
        rospy.spin()

if __name__ == "__main__":
    try:
        visualizer = RouteVisualizer()
        visualizer.run()
    except rospy.ROSInterruptException:
        pass