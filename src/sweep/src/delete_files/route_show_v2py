#!/usr/bin/env python3
import rospy
import yaml
import os
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import ColorRGBA

# 
#
#

class RouteVisualizer:
    def __init__(self):
        rospy.init_node('route_visualizer', anonymous=True)
        
        # 路径相关
        self.path_dir = os.path.join(os.path.dirname(__file__), "pathfiles")
        self.current_path = []
        self.current_pose_index = 0
        
        # 发布器
        self.path_pub = rospy.Publisher('/coverage_path', Path, queue_size=10)
        self.marker_pub = rospy.Publisher('/path_markers', MarkerArray, queue_size=10)
        self.points_pub = rospy.Publisher('/path_points', MarkerArray, queue_size=10)
        
        # 加载最新的路径文件
        self.load_latest_path()
        
        # 定时发布
        self.timer = rospy.Timer(rospy.Duration(0.1), self.publish_visualizations)

    def load_latest_path(self):
        """加载最新的路径文件"""
        try:
            # 获取最新的路径文件
            path_files = [f for f in os.listdir(self.path_dir) if f.startswith('coverage_path_') and f.endswith('.yaml')]
            if not path_files:
                rospy.logwarn("No path files found")
                return

            # 按文件名排序（因为我们用的是年月日时分格式）
            latest_file = sorted(path_files)[-1]  # 取最后一个（最新的）文件
            path_file = os.path.join(self.path_dir, latest_file)
            
            rospy.loginfo(f"Found these path files: {path_files}")  # 调试信息
            rospy.loginfo(f"Trying to load: {latest_file}")  # 调试信息
            
            with open(path_file, 'r') as f:
                path_data = yaml.safe_load(f)
                self.current_path = path_data['path_points']
                rospy.loginfo(f"Successfully loaded path from {path_file}")
                rospy.loginfo(f"Loaded {len(self.current_path)} points")  # 调试信息
                
        except Exception as e:
            rospy.logerr(f"Error loading path file: {e}")

    def create_path_message(self):
        """创建Path消息"""
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = rospy.Time.now()
        
        for point in self.current_path:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
            
        return path_msg

    def create_marker_array(self):
        """创建MarkerArray用于显示路径点和线段"""
        marker_array = MarkerArray()
        
        # 路径线段
        line_marker = Marker()
        line_marker.header.frame_id = "map"  # 确保使用map框架
        line_marker.header.stamp = rospy.Time.now()
        line_marker.ns = "path_lines"
        line_marker.id = 0
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.scale.x = 0.02  # 线宽
        line_marker.color = ColorRGBA(0.0, 0.5, 1.0, 0.8)  # 蓝色
        line_marker.pose.orientation.w = 1.0  # 添加这行
        
        # 添加路径点
        for point in self.current_path:
            p = Point()
            p.x = point[0]
            p.y = point[1]
            p.z = 0.0
            line_marker.points.append(p)
            
        marker_array.markers.append(line_marker)
        
        return marker_array

    def create_point_markers(self):
        """创建路径点标记"""
        marker_array = MarkerArray()
        
        for i, point in enumerate(self.current_path):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "path_points"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = point[0]
            marker.pose.position.y = point[1]
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 0.05
            
            # 根据是否已经经过设置不同颜色
            if i < self.current_pose_index:
                marker.color = ColorRGBA(0.0, 1.0, 0.0, 0.8)  # 绿色表示已经过
            else:
                marker.color = ColorRGBA(1.0, 0.0, 0.0, 0.8)  # 红色表示未经过
                
            marker_array.markers.append(marker)
            
        return marker_array

    def publish_visualizations(self, event):
        """发布所有可视化信息"""
        if not self.current_path:
            return
            
        self.path_pub.publish(self.create_path_message())
        self.marker_pub.publish(self.create_marker_array())
        self.points_pub.publish(self.create_point_markers())

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        visualizer = RouteVisualizer()
        visualizer.run()
    except rospy.ROSInterruptException:
        pass