#!/usr/bin/env python3
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker
from std_msgs.msg import String
import tf

class SweepController:
    def __init__(self):
        rospy.init_node('sweep_controller')
        
        # 订阅地图数据
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        
        # 发布路径
        self.path_pub = rospy.Publisher('/sweep_path', Path, queue_size=1)
        self.path_marker_pub = rospy.Publisher('/sweep_path_marker', Marker, queue_size=1)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        
        # 状态控制
        self.status_sub = rospy.Subscriber('/sweep_control', String, self.control_callback)
        
        # 存储数据
        self.map_data = None
        self.resolution = None
        self.origin = None
        self.current_path = None
        self.is_sweeping = False
        self.current_goal_index = 0

        rospy.loginfo("Sweep Controller initialized")

    def map_callback(self, map_msg):
        """处理地图数据"""
        self.map_data = np.array(map_msg.data).reshape(map_msg.info.height, map_msg.info.width)
        self.resolution = map_msg.info.resolution
        self.origin = map_msg.info.origin
        rospy.loginfo("Map received")

    def generate_sweep_path(self):
        """生成S型扫地路径"""
        if self.map_data is None:
            rospy.logwarn("No map data available")
            return None

        height, width = self.map_data.shape
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = rospy.Time.now()

        # S型路径生成
        step = int(0.5 / self.resolution)  # 0.5米间隔
        for i in range(0, height, step):
            row = i
            if (i // step) % 2 == 0:
                # 从左到右
                for j in range(0, width, step):
                    if self.is_valid_point(row, j):
                        pose = self.create_pose(row, j)
                        path.poses.append(pose)
            else:
                # 从右到左
                for j in range(width-1, -1, -step):
                    if self.is_valid_point(row, j):
                        pose = self.create_pose(row, j)
                        path.poses.append(pose)

        rospy.loginfo(f"Generated sweep path with {len(path.poses)} points")
        return path

    def is_valid_point(self, row, col):
        """检查点是否可通行"""
        if 0 <= row < self.map_data.shape[0] and 0 <= col < self.map_data.shape[1]:
            return self.map_data[row, col] == 0
        return False

    def create_pose(self, row, col):
        """创建位姿信息"""
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()
        
        # 转换地图坐标到真实世界坐标
        pose.pose.position.x = col * self.resolution + self.origin.position.x
        pose.pose.position.y = row * self.resolution + self.origin.position.y
        pose.pose.position.z = 0

        # 设置朝向（根据路径方向）
        pose.pose.orientation.w = 1.0
        return pose

    def visualize_path(self):
        """可视化路径"""
        if self.current_path is None:
            return

        # 发布Path消息
        self.path_pub.publish(self.current_path)

        # 创建路径标记
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "sweep_path"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.1  # 线宽
        marker.color.a = 1.0  # 不透明度
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        # 添加路径点
        for pose in self.current_path.poses:
            p = Point()
            p.x = pose.pose.position.x
            p.y = pose.pose.position.y
            p.z = pose.pose.position.z
            marker.points.append(p)

        self.path_marker_pub.publish(marker)

    def control_callback(self, msg):
        """处理控制命令"""
        command = msg.data.lower()
        
        if command == "start" and not self.is_sweeping:
            self.start_sweeping()
        elif command == "stop":
            self.stop_sweeping()

    def start_sweeping(self):
        """开始扫地"""
        rospy.loginfo("Starting sweeping")
        self.is_sweeping = True
        self.current_path = self.generate_sweep_path()
        if self.current_path:
            self.visualize_path()
            self.current_goal_index = 0
            self.send_next_goal()

    def stop_sweeping(self):
        """停止扫地"""
        rospy.loginfo("Stopping sweeping")
        self.is_sweeping = False
        self.current_goal_index = 0

    def send_next_goal(self):
        """发送下一个目标点"""
        if not self.is_sweeping or not self.current_path:
            return

        if self.current_goal_index < len(self.current_path.poses):
            goal = self.current_path.poses[self.current_goal_index]
            self.goal_pub.publish(goal)
            rospy.loginfo(f"Sending goal {self.current_goal_index + 1}/{len(self.current_path.poses)}")
            self.current_goal_index += 1
            # 等待一段时间后检查是否需要发送下一个目标
            rospy.Timer(rospy.Duration(5.0), self.check_goal_status, oneshot=True)

    def check_goal_status(self, event):
        """检查目标状态并决定是否发送下一个目标"""
        if self.is_sweeping:
            self.send_next_goal()

    def run(self):
        """主运行循环"""
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.current_path:
                self.visualize_path()
            rate.sleep()

if __name__ == '__main__':
    try:
        controller = SweepController()
        controller.run()
    except rospy.ROSInterruptException:
        pass