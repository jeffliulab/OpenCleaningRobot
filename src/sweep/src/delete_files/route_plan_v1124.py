#!/usr/bin/env python3
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
import yaml
import os
import cv2
import time
import matplotlib.pyplot as plt

class MapData:
    OBSTACLE = -1    # 障碍物点
    UNVISITED = 0    # 未访问的路径点
    VISITED = 1      # 已访问的路径点

class CoveragePathPlanner:
    def __init__(self):
        rospy.init_node('coverage_path_planner', anonymous=True)
        
        # 参数设置
        self.robot_radius = 0.2      # 机器人半径
        self.sweep_width = 0.4       # 清扫宽度
        self.grid_resolution = 0.05  # 栅格分辨率
        self.SAMPLING_INTERVAL = 5   # 采样间隔（每5个格子取一个点）
        
        # 地图信息
        self.map_origin_x = 0
        self.map_origin_y = 0
        self.map_resolution = 0.05
        self.map_height = 0
        self.map_width = 0
        
        # 地图数据
        self.map_array = None
        self.processed_map = None
        self.path_points = []
        
        # 创建目录
        script_dir = os.path.dirname(os.path.abspath(__file__))
        self.path_dir = os.path.join(script_dir, "pathfiles")
        self.debug_dir = os.path.join(script_dir, "debug")
        os.makedirs(self.path_dir, exist_ok=True)
        os.makedirs(self.debug_dir, exist_ok=True)
        
        # 订阅地图话题
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        rospy.loginfo("Coverage Path Planner initialized")

    def map_callback(self, map_data):
        """处理地图数据并进行路径规划"""
        # 保存地图信息
        self.map_origin_x = map_data.info.origin.position.x
        self.map_origin_y = map_data.info.origin.position.y
        self.map_resolution = map_data.info.resolution
        self.map_height = map_data.info.height
        self.map_width = map_data.info.width
        
        # 将地图转换为numpy数组
        self.map_array = np.array(map_data.data).reshape(
            map_data.info.height, map_data.info.width)
            
        rospy.loginfo(f"Map info - Size: {self.map_width}x{self.map_height}, "
                     f"Resolution: {self.map_resolution}, "
                     f"Origin: ({self.map_origin_x}, {self.map_origin_y})")
        
        # 进行路径规划
        self.plan_coverage_path()

    def preprocess_map(self, map_array):
        """将ROS地图转换为三值地图，并考虑安全距离"""
        rospy.loginfo("Preprocessing map...")
        processed_map = np.full_like(map_array, MapData.OBSTACLE, dtype=np.int8)
        
        # 设置可行区域
        processed_map[map_array == 0] = MapData.UNVISITED
        
        # 考虑安全距离的膨胀处理
        safety_kernel_size = int(2.0 * self.robot_radius / self.grid_resolution)
        safety_kernel = np.ones((safety_kernel_size, safety_kernel_size), np.uint8)
        
        # 对障碍物进行膨胀，标记安全区域
        obstacle_map = (processed_map == MapData.OBSTACLE).astype(np.uint8)
        dilated_obstacles = cv2.dilate(obstacle_map, safety_kernel, iterations=1)
        
        # 更新地图
        processed_map[dilated_obstacles == 1] = MapData.OBSTACLE
        
        # 保存为类成员变量，供其他函数使用
        self.processed_map = processed_map
        
        # 保存处理后的地图可视化
        self.save_processed_map_visualization(processed_map)
        
        return processed_map

    def save_processed_map_visualization(self, processed_map):
        """保存处理后的地图可视化"""
        plt.figure(figsize=(12, 12))
        
        # 创建显示用的地图
        display_map = np.full_like(processed_map, fill_value=1.0, dtype=float)
        display_map[processed_map == MapData.OBSTACLE] = 0.8
        
        plt.imshow(display_map, cmap='gray')
        plt.title("Processed Map")
        plt.colorbar(label='Map')
        
        timestamp = time.strftime("%Y%m%d%H%M")
        plt.savefig(os.path.join(self.debug_dir, f'processed_map_{timestamp}.png'))
        plt.close()

    def grid_to_world(self, grid_x, grid_y):
        """将栅格坐标转换为世界坐标"""
        world_x = grid_x * self.map_resolution + self.map_origin_x
        world_y = grid_y * self.map_resolution + self.map_origin_y
        return world_x, world_y

    def get_line_points(self, x1, y1, x2, y2):
        """使用Bresenham算法获取线段上的所有点"""
        points = []
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        x, y = x1, y1
        sx = 1 if x1 < x2 else -1
        sy = 1 if y1 < y2 else -1
        
        if dx > dy:
            err = dx / 2.0
            while x != x2:
                points.append((x, y))
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2.0
            while y != y2:
                points.append((x, y))
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy
                
        points.append((x, y))
        return points

    def line_crosses_obstacle(self, point1, point2):
        """检查两点之间的连线是否穿过障碍物"""
        x1, y1 = point1['grid_x'], point1['grid_y']
        x2, y2 = point2['grid_x'], point2['grid_y']
        
        points = self.get_line_points(x1, y1, x2, y2)
        
        for x, y in points:
            if not (0 <= x < self.map_width and 0 <= y < self.map_height):
                return True
            if self.processed_map[y, x] == MapData.OBSTACLE:
                return True
        return False

    def generate_path_points(self, processed_map):
        """生成采样后的路径点序列"""
        rospy.loginfo("Generating path points...")
        path_points = []
        height, width = processed_map.shape
        point_id = 0
        
        # 使用采样间隔生成路径点
        for x in range(0, width, self.SAMPLING_INTERVAL):
            if x >= width:
                break
            for y in range(0, height, self.SAMPLING_INTERVAL):
                if y >= height:
                    break
                    
                if processed_map[y, x] == MapData.UNVISITED:
                    world_x, world_y = self.grid_to_world(x, y)
                    path_points.append({
                        'id': point_id,
                        'grid_x': x,
                        'grid_y': y,
                        'world_x': world_x,
                        'world_y': world_y,
                        'status': MapData.UNVISITED
                    })
                    point_id += 1
        
        rospy.loginfo(f"Generated {len(path_points)} path points")
        return path_points

    def find_valid_connections(self, path_points):
        """找出所有有效的连接，确保每个点只被访问一次"""
        connections = []
        visited = set()
        
        current_point = path_points[0]
        visited.add(0)
        
        while len(visited) < len(path_points):
            best_distance = float('inf')
            best_next_point = None
            best_next_idx = None
            
            # 找到最近的未访问点
            for i, point in enumerate(path_points):
                if i in visited:
                    continue
                    
                if not self.are_adjacent(current_point, point):
                    continue
                    
                if self.line_crosses_obstacle(current_point, point):
                    continue
                
                distance = ((current_point['grid_x'] - point['grid_x']) ** 2 + 
                          (current_point['grid_y'] - point['grid_y']) ** 2) ** 0.5
                          
                if distance < best_distance:
                    best_distance = distance
                    best_next_point = point
                    best_next_idx = i
            
            if best_next_point is None:
                # 如果找不到下一个点，从剩余未访问点中选择一个新的起点
                unvisited = set(range(len(path_points))) - visited
                if unvisited:
                    next_start_idx = min(unvisited)
                    current_point = path_points[next_start_idx]
                    visited.add(next_start_idx)
                    continue
                else:
                    break
            
            connections.append((current_point, best_next_point))
            visited.add(best_next_idx)
            current_point = best_next_point
        
        return connections

    def are_adjacent(self, point1, point2):
        """判断两个点是否相邻"""
        dx = abs(point1['grid_x'] - point2['grid_x'])
        dy = abs(point1['grid_y'] - point2['grid_y'])
        return (dx == self.SAMPLING_INTERVAL and dy == 0) or (dx == 0 and dy == self.SAMPLING_INTERVAL)

    def visualize_plan(self, processed_map, path_points):
        """生成规划结果的可视化图片"""
        plt.figure(figsize=(12, 12))
        
        # 创建显示用的地图
        display_map = np.full_like(processed_map, fill_value=1.0, dtype=float)
        display_map[processed_map == MapData.OBSTACLE] = 0.8
        
        plt.imshow(display_map, cmap='gray')
        
        # 绘制路径点
        for point in path_points:
            plt.scatter(point['grid_x'], point['grid_y'], c='red', s=30)
        
        # 绘制连接
        connections = self.find_valid_connections(path_points)
        for point1, point2 in connections:
            plt.arrow(point1['grid_x'], point1['grid_y'],
                     point2['grid_x'] - point1['grid_x'],
                     point2['grid_y'] - point1['grid_y'],
                     head_width=2, head_length=2, fc='blue', ec='blue', alpha=0.5)
        
        plt.title(f'Coverage Path Plan (Interval={self.SAMPLING_INTERVAL})')
        plt.colorbar(label='Map (White=Free, Grey=Occupied)')
        
        timestamp = time.strftime("%Y%m%d%H%M")
        plot_file = os.path.join(self.debug_dir, f'path_plan_{timestamp}.png')
        plt.savefig(plot_file)
        plt.close()
        
        rospy.loginfo(f"Saved path visualization to {plot_file}")

    def plan_coverage_path(self):
        """主路径规划函数"""
        rospy.loginfo("Starting coverage path planning...")
        
        # 1. 预处理地图
        processed_map = self.preprocess_map(self.map_array)
        
        # 2. 生成路径点
        self.path_points = self.generate_path_points(processed_map)
        
        # 3. 生成可视化图片
        self.visualize_plan(processed_map, self.path_points)
        
        # 4. 保存路径
        self.save_path(processed_map)
        
        rospy.loginfo("Path planning completed")

    def save_path(self, processed_map):
        """保存规划的路径和地图数据"""
        if not self.path_points:
            rospy.logwarn("No path points to save")
            return
            
        timestamp = time.strftime("%Y%m%d%H%M", time.localtime())
        path_file = os.path.join(self.path_dir, f"coverage_path_{timestamp}.yaml")
        
        path_data = {
            'map_data': {
                'array': processed_map.tolist(),
                'width': self.map_width,
                'height': self.map_height,
                'resolution': self.map_resolution,
                'origin_x': self.map_origin_x,
                'origin_y': self.map_origin_y
            },
            'path_points': self.path_points,
            'metadata': {
                'robot_radius': self.robot_radius,
                'sweep_width': self.sweep_width,
                'sampling_interval': self.SAMPLING_INTERVAL,
                'timestamp': timestamp
            }
        }
        
        try:
            with open(path_file, 'w') as f:
                yaml.dump(path_data, f)
            rospy.loginfo(f"Path saved to {path_file}")
        except Exception as e:
            rospy.logerr(f"Failed to save path: {e}")

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        planner = CoveragePathPlanner()
        planner.run()
    except rospy.ROSInterruptException:
        pass