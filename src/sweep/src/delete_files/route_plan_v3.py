#!/usr/bin/env python3
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
import yaml
import os
import cv2
import time

# Update time: Nov 24, 2024

class MapData:
    OBSTACLE = -1    # 障碍物点
    UNVISITED = 0    # 未访问的路径点
    VISITED = 1      # 已访问的路径点

class CoveragePathPlanner:
    def __init__(self):
        rospy.init_node('coverage_path_planner', anonymous=True)
        
        # 参数设置
        self.robot_radius = 0.2  # 机器人半径
        self.sweep_width = 0.4   # 清扫宽度
        self.grid_resolution = 0.05  # 栅格分辨率
        
        # 地图信息
        self.map_origin_x = 0
        self.map_origin_y = 0
        self.map_resolution = 0.05
        self.map_height = 0
        self.map_width = 0
        
        # 保存路径的目录
        script_dir = os.path.dirname(os.path.abspath(__file__))
        self.path_dir = os.path.join(script_dir, "pathfiles")
        os.makedirs(self.path_dir, exist_ok=True)
        
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
        # 完成后关闭节点
        rospy.signal_shutdown("Planning completed")

    def preprocess_map(self, map_array):
        """将ROS地图转换为我们的三值地图"""
        processed_map = np.full_like(map_array, MapData.OBSTACLE, dtype=np.int8)
        
        # 设置可行区域为UNVISITED
        processed_map[map_array == 0] = MapData.UNVISITED
        
        # 膨胀处理以考虑机器人半径
        kernel_size = int(1.5 * self.robot_radius / self.grid_resolution)
        kernel = np.ones((kernel_size, kernel_size), np.uint8)
        
        # 将地图转换为二值图进行膨胀
        binary_map = (processed_map == MapData.UNVISITED).astype(np.uint8)
        eroded_map = cv2.erode(binary_map, kernel, iterations=1)
        
        # 更新处理后的地图
        processed_map[binary_map != eroded_map] = MapData.OBSTACLE
        
        return processed_map

    def generate_path_points(self, processed_map):
        """生成路径点序列"""
        path_points = []
        sequence = []  # 存储路径点的访问顺序
        height, width = processed_map.shape
        
        # 计算扫描线间距（像素单位）
        line_spacing = int(self.sweep_width / self.grid_resolution)
        
        # 生成Z字形路径
        point_id = 0  # 用于标记点的访问顺序
        going_up = True
        for x in range(0, width, line_spacing):
            if x >= width:
                break
                
            current_line = []
            if going_up:
                for y in range(height):
                    if processed_map[y, x] == MapData.UNVISITED:
                        world_x, world_y = self.grid_to_world(x, y)
                        current_line.append({
                            'id': point_id,
                            'grid': (x, y),
                            'world': (world_x, world_y)
                        })
                        point_id += 1
            else:
                for y in range(height-1, -1, -1):
                    if processed_map[y, x] == MapData.UNVISITED:
                        world_x, world_y = self.grid_to_world(x, y)
                        current_line.append({
                            'id': point_id,
                            'grid': (x, y),
                            'world': (world_x, world_y)
                        })
                        point_id += 1
            
            path_points.extend(current_line)
            going_up = not going_up
        
        return path_points

    def grid_to_world(self, grid_x, grid_y):
        """将栅格坐标转换为世界坐标"""
        world_x = grid_x * self.map_resolution + self.map_origin_x
        world_y = grid_y * self.map_resolution + self.map_origin_y
        return world_x, world_y

    def plan_coverage_path(self):
        """主路径规划函数"""
        rospy.loginfo("Starting coverage path planning...")
        
        # 1. 预处理地图
        processed_map = self.preprocess_map(self.map_array)
        
        # 2. 生成路径点
        path_points = self.generate_path_points(processed_map)
        
        # 3. 保存结果
        self.save_results(processed_map, path_points)

    def save_results(self, processed_map, path_points):
        """保存处理后的地图和路径点"""
        if not path_points:
            rospy.logwarn("No path points to save")
            return
            
        timestamp = time.strftime("%Y%m%d%H%M", time.localtime())
        path_file = os.path.join(self.path_dir, f"coverage_path_{timestamp}.yaml")
        
        # 提取需要保存的数据
        save_data = {
            'map_data': {
                'array': processed_map.tolist(),
                'origin_x': self.map_origin_x,
                'origin_y': self.map_origin_y,
                'resolution': self.map_resolution,
                'height': self.map_height,
                'width': self.map_width
            },
            'path_points': [
                {
                    'id': point['id'],
                    'grid_x': point['grid'][0],
                    'grid_y': point['grid'][1],
                    'world_x': point['world'][0],
                    'world_y': point['world'][1]
                }
                for point in path_points
            ],
            'metadata': {
                'robot_radius': self.robot_radius,
                'sweep_width': self.sweep_width,
                'timestamp': timestamp
            }
        }
        
        try:
            with open(path_file, 'w') as f:
                yaml.dump(save_data, f)
            rospy.loginfo(f"Results saved to {path_file}")
        except Exception as e:
            rospy.logerr(f"Failed to save results: {e}")

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        planner = CoveragePathPlanner()
        planner.run()
    except rospy.ROSInterruptException:
        pass