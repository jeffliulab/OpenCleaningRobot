#!/usr/bin/env python3
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
import yaml
import os
import cv2
import time
from queue import PriorityQueue
import math

# Update Time: Nov22, 2024
# Needs Update: 
# now the red points are correct
# but when analyze the route
# it just simply connect all the points
# and make the route unreliable
# so what needs to do next:
# (1) understand the red points
# (2) think how to use A* to plan a route, and avoid obstacles and unreachable areas
# (3) connect those points
# 

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
        
        # Debug: 保存处理后的地图的目录
        self.debug_dir = os.path.join(script_dir, "debug_maps")
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
        # 完成后关闭节点
        rospy.signal_shutdown("Planning completed")

    def detect_outer_boundary(self, binary_map):
        """检测地图的外围边界"""
        height, width = binary_map.shape
        boundary_points = []
        
        # 从外向内扫描，找到第一个障碍物点
        # 上边界
        for x in range(width):
            for y in range(height):
                if binary_map[y, x] == 0:  # 找到障碍物
                    boundary_points.append((x, y))
                    break
        
        # 右边界
        for y in range(height):
            for x in range(width-1, -1, -1):
                if binary_map[y, x] == 0:
                    boundary_points.append((x, y))
                    break
        
        # 下边界
        for x in range(width-1, -1, -1):
            for y in range(height-1, -1, -1):
                if binary_map[y, x] == 0:
                    boundary_points.append((x, y))
                    break
        
        # 左边界
        for y in range(height-1, -1, -1):
            for x in range(width):
                if binary_map[y, x] == 0:
                    boundary_points.append((x, y))
                    break
        
        return boundary_points

    def detect_closed_regions(self, binary_map):
        """检测并填充封闭的障碍物区域，使用更宽松的标准"""
        # 确保地图是uint8类型
        binary_map_uint8 = binary_map.astype(np.uint8)
        
        # 使用OpenCV的findContours找到所有轮廓
        contours, _ = cv2.findContours(
            (1 - binary_map_uint8),  # 取反，使障碍物为白色
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )
        
        # 填充所有封闭轮廓
        filled_map = binary_map_uint8.copy()
        for contour in contours:
            # 增加面积阈值，只处理较大的封闭区域
            area = cv2.contourArea(contour)
            if area > 100:  # 显著增加阈值
                # 计算轮廓的周长
                perimeter = cv2.arcLength(contour, True)
                # 使用更宽松的多边形近似
                epsilon = 0.1 * perimeter
                approx = cv2.approxPolyDP(contour, epsilon, True)
                
                # 只填充多边形点数较多的区域
                if len(approx) > 4:
                    cv2.drawContours(filled_map, [contour], -1, 0, -1)
        
        # Debug: 保存处理后的地图
        debug_path = os.path.join(self.debug_dir, 'closed_regions.png')
        cv2.imwrite(debug_path, filled_map * 255)
        
        return filled_map.astype(bool)  # 转回布尔类型

    def preprocess_map(self, map_array):
        """改进的地图预处理函数"""
        rospy.loginfo("Starting map preprocessing...")
        
        # 1. 基本二值化
        binary_map = np.zeros_like(map_array)
        binary_map[map_array == 0] = 1    # 确定的空闲区域
        binary_map[map_array != 0] = 0    # 其他都视为障碍物
        
        # 输出初始二值化后的可行点数量
        free_cells = np.sum(binary_map == 1)
        rospy.loginfo(f"Initial free cells: {free_cells}")
        
        # Debug: 保存初始二值化地图
        debug_path = os.path.join(self.debug_dir, 'initial_binary.png')
        cv2.imwrite(debug_path, binary_map.astype(np.uint8) * 255)
        
        # 2. 处理外围边界
        boundary = self.detect_outer_boundary(binary_map)
        rospy.loginfo(f"Detected {len(boundary)} boundary points")
        
        # 3. 处理封闭障碍物区域
        map_with_regions = self.detect_closed_regions(binary_map)
        free_cells_after_regions = np.sum(map_with_regions == 1)
        rospy.loginfo(f"Free cells after region detection: {free_cells_after_regions}")
        
        # 4. 最终膨胀处理
        kernel_size = int(1.5 * self.robot_radius / self.grid_resolution)
        kernel = np.ones((kernel_size, kernel_size), np.uint8)
        final_map = cv2.erode(map_with_regions.astype(np.uint8), kernel, iterations=1)
        
        free_cells_final = np.sum(final_map == 1)
        rospy.loginfo(f"Final free cells: {free_cells_final}")
        
        # Debug: 保存最终处理后的地图
        debug_path = os.path.join(self.debug_dir, 'final_processed.png')
        cv2.imwrite(debug_path, final_map * 255)
        
        rospy.loginfo("Map preprocessing completed")
        return final_map.astype(bool)

    def grid_to_world(self, grid_x, grid_y):
        """将栅格坐标转换为世界坐标"""
        world_x = grid_x * self.map_resolution + self.map_origin_x
        world_y = grid_y * self.map_resolution + self.map_origin_y
        return world_x, world_y

    def world_to_grid(self, world_x, world_y):
        """将世界坐标转换为栅格坐标"""
        grid_x = int((world_x - self.map_origin_x) / self.map_resolution)
        grid_y = int((world_y - self.map_origin_y) / self.map_resolution)
        return (grid_x, grid_y)

    def get_neighbors(self, current, processed_map):
        """获取邻近的可行点"""
        x, y = current
        neighbors = []
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0),
                     (1, 1), (-1, 1), (1, -1), (-1, -1)]  # 8个方向
        
        for dx, dy in directions:
            new_x, new_y = x + dx, y + dy
            if (0 <= new_x < processed_map.shape[1] and 
                0 <= new_y < processed_map.shape[0] and 
                processed_map[new_y, new_x] == 1):
                neighbors.append((new_x, new_y))
        
        return neighbors

    def heuristic(self, a, b):
        """A*算法的启发式函数，使用欧几里得距离"""
        return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

    def find_safe_path(self, start_point, end_point, processed_map):
        """使用A*算法找到两点之间的安全路径"""
        # 将世界坐标转换回栅格坐标
        start_grid = self.world_to_grid(start_point[0], start_point[1])
        end_grid = self.world_to_grid(end_point[0], end_point[1])
        
        # A*算法实现
        frontier = PriorityQueue()
        frontier.put((0, start_grid))
        came_from = {start_grid: None}
        cost_so_far = {start_grid: 0}
        
        while not frontier.empty():
            current = frontier.get()[1]
            
            if current == end_grid:
                break
            
            for next_pos in self.get_neighbors(current, processed_map):
                new_cost = cost_so_far[current] + 1  # 假设每一步代价为1
                
                if next_pos not in cost_so_far or new_cost < cost_so_far[next_pos]:
                    cost_so_far[next_pos] = new_cost
                    priority = new_cost + self.heuristic(end_grid, next_pos)
                    frontier.put((priority, next_pos))
                    came_from[next_pos] = current
        
        # 重建路径
        path = []
        current = end_grid
        while current is not None:
            path.append(current)
            current = came_from.get(current)
        path.reverse()
        
        # 将路径转换回世界坐标
        return [self.grid_to_world(x, y) for x, y in path]

    def generate_coverage_path(self, processed_map):
        """生成带有安全连接的覆盖路径"""
        rospy.loginfo("Generating coverage path...")
        path_points = []
        height, width = processed_map.shape
        
        # 计算扫描线间距（像素单位）
        line_spacing = int(self.sweep_width / self.map_resolution)
        
        # 生成Z字形路径
        going_up = True
        last_valid_point = None
        
        for x in range(0, width, line_spacing):
            if x >= width:
                break
                
            current_line_points = []
            if going_up:
                for y in range(height):
                    if processed_map[y, x] == 1:
                        world_x, world_y = self.grid_to_world(x, y)
                        current_line_points.append([world_x, world_y])
            else:
                for y in range(height-1, -1, -1):
                    if processed_map[y, x] == 1:
                        world_x, world_y = self.grid_to_world(x, y)
                        current_line_points.append([world_x, world_y])
            
            # 如果当前列有点
            if current_line_points:
                if last_valid_point is not None:
                    # 计算安全连接路径
                    safe_path = self.find_safe_path(
                        last_valid_point,
                        current_line_points[0],
                        processed_map
                    )
                    path_points.extend(safe_path)
                
                path_points.extend(current_line_points)
                last_valid_point = current_line_points[-1]
            
            going_up = not going_up
        
        rospy.loginfo(f"Generated path with {len(path_points)} points")
        return path_points

    def plan_coverage_path(self):
        """主路径规划函数"""
        rospy.loginfo("Starting coverage path planning...")
        
        # 1. 预处理地图
        processed_map = self.preprocess_map(self.map_array)
        
        # 2. 生成覆盖路径
        self.path_points = self.generate_coverage_path(processed_map)
        
        # 3. 保存路径
        self.save_path()

    def save_path(self):
        """保存规划的路径"""
        if not self.path_points:
            rospy.logwarn("No path points to save")
            return
            
        # 确保所有点都是列表格式，而不是元组
        formatted_points = [[float(x), float(y)] for x, y in self.path_points]
            
        # 使用年月日时分格式命名
        timestamp = time.strftime("%Y%m%d%H%M", time.localtime())
        path_file = os.path.join(self.path_dir, f"coverage_path_{timestamp}.yaml")
        
        path_data = {
            'path_points': formatted_points,  # 使用格式化后的点
            'metadata': {
                'robot_radius': float(self.robot_radius),
                'sweep_width': float(self.sweep_width),
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
        """运行路径规划器"""
        rospy.spin()

if __name__ == "__main__":
    try:
        planner = CoveragePathPlanner()
        planner.run()
    except rospy.ROSInterruptException:
        pass