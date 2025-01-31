#!/usr/bin/env python3

import rospy
import subprocess
import signal
from std_msgs.msg import String

# Update: Nov 25, 2024
# Need update:
# "quit" command seems not useful now, because now use quit button to kill all, instead of kill one by one
# if this doesn't use for long, please delete it.

class GmappingController:
    def __init__(self):
        # Initilization
        rospy.init_node("gmapping_controller", anonymous=True)
        
        # gmapping process
        self.gmapping_process = None
        
        # subscribe topic
        rospy.Subscriber("/control_gmapping", String, self.control_callback)
        
        rospy.loginfo("Gmapping Controller is running. Waiting for commands...")

    def control_callback(self, msg):
        # 接收控制命令，解析动作
        if msg.data == "start_gmapping":
            self.start_gmapping()
        elif msg.data == "stop_gmapping":
            self.stop_gmapping()
        elif msg.data == "quit":
            rospy.loginfo("Exiting Gmapping Controller.")
            if self.gmapping_process is not None:
                self.stop_gmapping()  # 确保退出时关闭 gmapping
            rospy.signal_shutdown("User requested shutdown")

    def start_gmapping(self):
        if self.gmapping_process is None:
            rospy.loginfo("Starting turtlebot3_slam with gmapping...")

            # 启动 gmapping.launch 子进程 || Nov25 debug: add "robot_state_publisher:=false" to debug (test)
            self.gmapping_process = subprocess.Popen([
                "roslaunch", 
                "turtlebot3_slam", 
                "turtlebot3_slam.launch", 
                "slam_methods:=gmapping"
                ])
        else:
            rospy.loginfo("Gmapping is already running.")

    def stop_gmapping(self):
        if self.gmapping_process is not None:
            rospy.loginfo("Stopping gmapping.launch...")
            # 发送 SIGINT 信号（模拟 Ctrl+C）来终止进程
            self.gmapping_process.send_signal(signal.SIGINT)
            self.gmapping_process = None
        else:
            rospy.loginfo("Gmapping is not running.")

    def run(self):
        rospy.spin()  # 保持节点运行，监听话题

if __name__ == "__main__":
    controller = GmappingController()
    controller.run()
