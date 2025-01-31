#!/usr/bin/env python3
import rospy
import yaml
import os
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

class MapData:
    OBSTACLE = -1
    UNVISITED = 0
    VISITED = 1

class PointDisplay:
    OBSTACLE_COLOR = ColorRGBA(0.0, 0.0, 0.0, 0.8)    # 黑色
    UNVISITED_COLOR = ColorRGBA(1.0, 0.0, 0.0, 0.8)   # 红色
    VISITED_COLOR = ColorRGBA(0.0, 1.0, 0.0, 0.8)     # 绿色
    POINT_SIZE = 0.05

class RouteVisualizer:
    def __init__(self):
        rospy.init_node('route_visualizer', anonymous=True)
        
        # 数据存储
        self.map_data = None
        self.path_points = []
        self.current_point_id = 0  # 当前访问点的ID
        
        # 发布器
        self.points_pub = rospy.Publisher('/path_visualization', MarkerArray, queue_size=10)
        self.connection_pub = rospy.Publisher('/path_connections', MarkerArray, queue_size=10)
        
        # 加载路径文件
        self.path_dir = os.path.join(os.path.dirname(__file__), "pathfiles")
        self.load_latest_path()
        
        # 定时发布
        self.timer = rospy.Timer(rospy.Duration(0.1), self.publish_visualizations)

    def load_latest_path(self):
        """加载最新的路径文件"""
        try:
            path_files = [f for f in os.listdir(self.path_dir) if f.startswith('coverage_path_') and f.endswith('.yaml')]
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
            rospy.logerr(f"Error loading path file: {e}")

    def create_point_marker(self, point_info, index, point_type):
        """创建点标记"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "path_points"
        marker.id = index
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        # 设置位置
        marker.pose.position.x = point_info['world_x']
        marker.pose.position.y = point_info['world_y']
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        
        # 设置大小
        marker.scale.x = marker.scale.y = marker.scale.z = PointDisplay.POINT_SIZE
        
        # 根据点类型设置颜色
        if point_type == MapData.OBSTACLE:
            marker.color = PointDisplay.OBSTACLE_COLOR
        elif point_type == MapData.VISITED:
            marker.color = PointDisplay.VISITED_COLOR
        else:
            marker.color = PointDisplay.UNVISITED_COLOR
            
        return marker

    def create_connection_markers(self):
        """创建相邻点之间的连接线"""
        markers = []
        marker_id = 0
        
        for i, point in enumerate(self.path_points):
            # 检查下一个点
            if i + 1 < len(self.path_points):
                next_point = self.path_points[i + 1]
                
                # 检查是否为相邻点（只考虑上下左右）
                dx = abs(next_point['grid_x'] - point['grid_x'])
                dy = abs(next_point['grid_y'] - point['grid_y'])
                
                if (dx == 0 and dy == 1) or (dx == 1 and dy == 0):
                    marker = Marker()
                    marker.header.frame_id = "map"
                    marker.header.stamp = rospy.Time.now()
                    marker.ns = "connections"
                    marker.id = marker_id
                    marker.type = Marker.ARROW
                    marker.action = Marker.ADD
                    
                    # 设置起点和终点
                    start = Point()
                    start.x = point['world_x']
                    start.y = point['world_y']
                    
                    end = Point()
                    end.x = next_point['world_x']
                    end.y = next_point['world_y']
                    
                    marker.points = [start, end]
                    
                    # 设置箭头的大小
                    marker.scale.x = 0.02  # 箭身宽度
                    marker.scale.y = 0.04  # 箭头宽度
                    
                    # 设置颜色（使用浅蓝色）
                    marker.color = ColorRGBA(0.3, 0.3, 1.0, 0.8)
                    
                    markers.append(marker)
                    marker_id += 1
        
        return markers

    def publish_visualizations(self, event):
        """发布可视化信息"""
        if not self.path_points:
            return
        
        # 创建点标记
        point_markers = MarkerArray()
        
        # 添加路径点
        for i, point in enumerate(self.path_points):
            # 根据点ID判断是否已访问
            point_type = MapData.VISITED if point['id'] < self.current_point_id else MapData.UNVISITED
            marker = self.create_point_marker(point, i, point_type)
            point_markers.markers.append(marker)
        
        # 添加障碍物点
        obstacle_id = len(self.path_points)
        for y in range(self.map_data['height']):
            for x in range(self.map_data['width']):
                if self.map_data['array'][y][x] == MapData.OBSTACLE:
                    #world_x = x * self.map_data['resolution
                    # Claude stops here                            
                    # followings are finished by ChatGPT

                    world_x = x * self.map_data['resolution'] + self.map_data['origin']['x']
                    world_y = y * self.map_data['resolution'] + self.map_data['origin']['y']
                    
                    # 创建障碍物点标记
                    marker = Marker()
                    marker.header.frame_id = "map"
                    marker.header.stamp = rospy.Time.now()
                    marker.ns = "obstacles"
                    marker.id = obstacle_id
                    marker.type = Marker.SPHERE
                    marker.action = Marker.ADD
                    
                    # 设置位置
                    marker.pose.position.x = world_x
                    marker.pose.position.y = world_y
                    marker.pose.position.z = 0.0
                    marker.pose.orientation.w = 1.0
                    
                    # 设置大小
                    marker.scale.x = marker.scale.y = marker.scale.z = PointDisplay.POINT_SIZE
                    
                    # 设置颜色
                    marker.color = PointDisplay.OBSTACLE_COLOR
                    
                    point_markers.markers.append(marker)
                    obstacle_id += 1
        
        # 发布点标记
        self.points_pub.publish(point_markers)
        
        # 创建并发布连接线标记
        connection_markers = MarkerArray()
        connection_markers.markers = self.create_connection_markers()
        self.connection_pub.publish(connection_markers)

if __name__ == "__main__":
    try:
        visualizer = RouteVisualizer()
        rospy.spin()  # 保持节点运行
    except rospy.ROSInterruptException:
        rospy.loginfo("Route visualizer node terminated.")
