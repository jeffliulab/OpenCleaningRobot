#!/usr/bin/env python3
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
import yaml
import os
import cv2

# Nov22, this version cannot use to following, but can work to show a route

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
        
        # 保存路径的目录
        self.path_dir = os.path.join(os.path.dirname(__file__), "pathfiles")
        os.makedirs(self.path_dir, exist_ok=True)
        
        # 订阅地图话题
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        
        # 路径规划结果
        self.path_points = []
        
        rospy.loginfo("Coverage Path Planner initialized")
        
    def map_callback(self, map_data):
        """处理地图数据并进行路径规划"""
        # 保存地图信息
        self.map_origin_x = map_data.info.origin.position.x
        self.map_origin_y = map_data.info.origin.position.y
        self.map_resolution = map_data.info.resolution
        
        # 将地图转换为numpy数组
        self.map_array = np.array(map_data.data).reshape(
            map_data.info.height, map_data.info.width)
        
        rospy.loginfo(f"Map info - Origin: ({self.map_origin_x}, {self.map_origin_y}), Resolution: {self.map_resolution}")
        rospy.loginfo(f"Map size: {map_data.info.width} x {map_data.info.height}")
        
        # 进行路径规划
        self.plan_coverage_path(map_data)
        
    def grid_to_world(self, grid_x, grid_y):
        """将栅格坐标转换为世界坐标"""
        world_x = grid_x * self.map_resolution + self.map_origin_x
        world_y = grid_y * self.map_resolution + self.map_origin_y
        return world_x, world_y

    def preprocess_map(self, map_array):
        """地图预处理，转换为二值图，采用更严格的策略"""
        # 创建二值地图
        binary_map = np.zeros_like(map_array)
        
        # 只有完全确定的空闲区域(0)才标记为可行
        # 添加调试信息
        rospy.loginfo(f"Unique values in map: {np.unique(map_array)}")
        
        # 更严格的条件：
        threshold = 10  # 如果大于这个值就认为是障碍物
        binary_map[(map_array == 0)] = 1  # 只有确定是0的才是可行区域
        binary_map[(map_array > threshold) | (map_array == -1)] = 0  # 其他都是障碍物
        
        # 增加膨胀处理的范围
        kernel_size = int(2.0 * self.robot_radius / self.grid_resolution)  # 增加安全系数
        kernel = np.ones((kernel_size, kernel_size))
        binary_map = cv2.erode(binary_map.astype(np.uint8), kernel, iterations=1)
        
        # 添加调试信息
        non_zero = np.count_nonzero(binary_map)
        total = binary_map.size
        rospy.loginfo(f"Traversable area: {non_zero}/{total} pixels ({non_zero/total*100:.2f}%)")
        
        # 可视化处理后的地图（可选）
        if rospy.get_param('~debug', False):
            cv2.imwrite('binary_map.png', binary_map * 255)
        
        return binary_map
        
    def decompose_regions(self, binary_map):
        """将地图分解为可清扫的子区域"""
        # 使用连通区域分析
        num_labels, labels = cv2.connectedComponents(binary_map)
        
        regions = []
        for i in range(1, num_labels):
            region = (labels == i).astype(np.uint8)
            regions.append(region)
            
        rospy.loginfo(f"Decomposed into {len(regions)} regions")
        return regions
        
    def generate_zigzag_path(self, region, resolution, sweep_width):
        """生成Z字形覆盖路径"""
        path_points = []
        height, width = region.shape
        
        # 计算扫描线间距（像素单位）
        line_spacing = int(sweep_width / resolution)
        
        # 生成Z字形路径
        going_up = True
        for x in range(0, width, line_spacing):
            if x >= width:
                break
                
            if going_up:
                for y in range(height):
                    if region[y, x] == 1:
                        # 转换为世界坐标
                        world_x, world_y = self.grid_to_world(x, y)
                        path_points.append([world_x, world_y])
            else:
                for y in range(height-1, -1, -1):
                    if region[y, x] == 1:
                        # 转换为世界坐标
                        world_x, world_y = self.grid_to_world(x, y)
                        path_points.append([world_x, world_y])
            
            going_up = not going_up
            
        return path_points

    def plan_coverage_path(self, map_data):
        """实现基于Boustrophedon的覆盖路径规划"""
        rospy.loginfo("Starting coverage path planning...")
        
        # 1. 地图预处理
        binary_map = self.preprocess_map(self.map_array)
        
        # 2. 区域分解
        regions = self.decompose_regions(binary_map)
        
        # 3. 为每个区域生成Z字形路径
        all_path_points = []
        for i, region in enumerate(regions):
            rospy.loginfo(f"Planning path for region {i+1}/{len(regions)}")
            sub_path = self.generate_zigzag_path(
                region, 
                map_data.info.resolution,
                self.sweep_width
            )
            all_path_points.extend(sub_path)
        
        self.path_points = all_path_points
        rospy.loginfo(f"Path planning completed with {len(self.path_points)} points")
        
        # 4. 保存路径
        self.save_path()
        
    def save_path(self):
        """保存规划的路径"""
        if not self.path_points:
            rospy.logwarn("No path points to save")
            return
            
        path_file = os.path.join(self.path_dir, 
            f"coverage_path_{rospy.Time.now().to_sec():.0f}.yaml")
        
        path_data = {
            'path_points': self.path_points,
            'metadata': {
                'robot_radius': self.robot_radius,
                'sweep_width': self.sweep_width,
                'timestamp': rospy.Time.now().to_sec()
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