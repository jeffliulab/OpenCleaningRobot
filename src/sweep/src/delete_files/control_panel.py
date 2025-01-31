#!/usr/bin/env python3

#
#
#
# LAST UPDATE TIME: Nov 22, 2024
# FROM THEN ON
# ALL MODIFICATION IS IN control_panel_gui.py
# THIS SCRIPT IS FOR BACKUP
#
#
#
# 
#
#
#
#
#
#
#
#
#
#
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import PoseStamped
import sys
import select
import termios
import tty
import subprocess
import time
import os

class ControlPanel:
    def __init__(self):
        rospy.init_node("control_panel_node", anonymous=True)
        rospy.sleep(1)  # 确保所有初始化完成
        
        # 发布者
        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        self.control_move_base_pub = rospy.Publisher("/control_move_base", String, queue_size=10)
        self.control_explore_pub = rospy.Publisher("/control_explore", String, queue_size=10)
        self.velocity_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.cancel_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=10)
        
        self.control_gmapping_pub = rospy.Publisher("/control_gmapping", String, queue_size=10)
        self.control_amcl_pub = rospy.Publisher("/control_amcl", String, queue_size=10)





        # 订阅语音命令
        rospy.Subscriber("voice_commands", String, self.voice_callback)

        self.running = True  # 控制主循环是否继续的标志

        # 提示用户操作
        self.print_instructions()

    def print_instructions(self):
        """打印控制面板说明"""
        print("CONTROL PANEL")
        print("=====EXPLORE CONTROL=====")
        print("INSTRUCTIONS:")
        print("If you want to build a map")
        print("PRESS '6' TO START SLAM")
        print("PRESS '1' or SPEAK 'START' TO START EXPLORATION AND MOVEBASE")
        print("PRESS '2' or SPEAK 'STOP' TO STOP EXPLORATION, MOVEBASE AND STOP THE ROBOT")
        print("PRESS 's' TO SAVE THE MAP")
        print("PRESS 'q' or SPEAK 'QUIT PROGRAM' TO EXIT THE PROGRAM")
        print("=====SWEEP MODULE======")
        print("PRESS '7' TO STOP SLAM")
        print("PRESS '8' TO START AMCL") # do not use move_base anymore, instead use DIY algorithm
        print("PRESS '9' TO STOP AMCL")

    def voice_callback(self, msg):
        """语音命令回调函数"""
        command = msg.data.lower()

        if "start" in command:
            rospy.loginfo(f"Received voice command: {command}")
            self.start_control()
        elif "stop" in command:
            rospy.loginfo(f"Received voice command: {command}")
            self.stop_control()
        elif "quit program" in command:
            rospy.loginfo(f"Received voice command: {command}")
            self.shutdown()

    def start_control(self):
        """启动控制模块"""
        self.control_explore_pub.publish("start_explore")
        rospy.loginfo("Published: start_explore")

        self.control_move_base_pub.publish("start_move_base")
        rospy.loginfo("Published: start_move_base")

    def stop_control(self):
        """关闭控制模块"""
        self.control_explore_pub.publish("stop_explore")
        rospy.loginfo("Published: stop_explore")

        self.control_move_base_pub.publish("stop_move_base")
        rospy.loginfo("Published: stop_move_base")

        # 发送零速度指令停止机器人
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.velocity_pub.publish(stop_msg)
        rospy.loginfo("Published: stop velocity to halt the robot")

        # 发送取消指令给 move_base
        cancel_msg = GoalID()
        self.cancel_pub.publish(cancel_msg)
        rospy.loginfo("Published: cancel navigation goal to stop move_base")

    def save_map(self):
        """保存地图"""
        map_path = "~/clean_robot_119/cleaning_robot/maps/map_" + time.strftime("%Y%m%d_%H%M%S")
        try:
            subprocess.call(["rosrun", "map_server", "map_saver", "-f", os.path.expanduser(map_path)])
            rospy.loginfo(f"Map saved to {map_path}")
        except Exception as e:
            rospy.logerr(f"Failed to save map: {e}")


    def shutdown(self):
        """停止程序"""
        rospy.loginfo("Shutting down the control panel...")
        self.running = False
        rospy.signal_shutdown("User requested shutdown")

    def get_key(self):
        """非阻塞读取按键输入"""
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = None
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin))
        return key

    def run(self):
        """主运行逻辑"""
        print("Listening for key presses...")
        while not rospy.is_shutdown() and self.running:
            key = self.get_key()

            if key == '1':  # 按键 '1'
                rospy.loginfo("Key '1' pressed.")
                self.start_control()

            elif key == '2':  # 按键 '2'
                rospy.loginfo("Key '2' pressed.")
                self.stop_control()

            elif key == 'q':  # 按键 'q'
                rospy.loginfo("Key 'q' pressed.")
                self.shutdown()
                break

            elif key == 's': 
                rospy.loginfo("Key 's' pressed.")
                self.save_map()

            elif key == '6':  # 按键 '6' 启动 gmapping
                rospy.loginfo("Key '6' pressed. Starting gmapping.")
                self.control_gmapping_pub.publish("start_gmapping")

            elif key == '7':  # 按键 '7' 停止 gmapping
                rospy.loginfo("Key '7' pressed. Stopping gmapping.")
                self.control_gmapping_pub.publish("stop_gmapping")

            elif key == '8':  # 按键 '8' 启动 AMCL 
                rospy.loginfo("Key '8' pressed. Starting AMCL.")
                self.control_amcl_pub.publish("start_amcl")

            elif key == '9':  # 按键 '9' 停止 AMCL 
                rospy.loginfo("Key '9' pressed. Stopping AMCL.")
                self.control_amcl_pub.publish("stop_amcl")


            rospy.sleep(0.1)  # 降低 CPU 占用率


if __name__ == "__main__":
    control_panel = ControlPanel()
    control_panel.run()
