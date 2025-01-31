#!/usr/bin/env python3
import rospy
import subprocess
import signal
from std_msgs.msg import String

# Latest Update: Nov25, 2024

class RouteShowController:
    def __init__(self):
        rospy.init_node("route_show_controller", anonymous=True)
        
        # 存储进程
        self.show_process = None
        
        # 订阅控制话题
        rospy.Subscriber("/control_route_show", String, self.control_callback)
        
        rospy.loginfo("Route Show Controller is running. Waiting for commands...")

    def control_callback(self, msg):
        if msg.data == "start_show":
            self.start_show()
        elif msg.data == "stop_show":
            self.stop_show()
        elif msg.data == "quit":
            rospy.loginfo("Exiting Route Show Controller.")
            if self.show_process is not None:
                self.stop_show()
            rospy.signal_shutdown("User requested shutdown")

    def start_show(self):
        if self.show_process is None:
            rospy.loginfo("Starting route_show using rosrun...")
            try:
                # 使用rosrun启动route_show.py
                self.show_process = subprocess.Popen(
                    ["rosrun", "sweep", "route_show.py"],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE
                )
                rospy.loginfo("Route show started successfully")
                rospy.loginfo("Successfully Publish the Route")
                rospy.loginfo("If you don't see the route, please open those in RViz:")
                rospy.loginfo("path, /coverage_path")
                rospy.loginfo("markerarray, /path_markers")
                rospy.loginfo("markerarray, /path_points")
            except Exception as e:
                rospy.logerr(f"Failed to start route show: {e}")
        else:
            rospy.loginfo("Route show is already running.")

    def stop_show(self):
        if self.show_process is not None:
            rospy.loginfo("Stopping route show...")
            try:
                self.show_process.send_signal(signal.SIGINT)
                self.show_process.wait(timeout=5)
                rospy.loginfo("Route show stopped successfully")
            except subprocess.TimeoutExpired:
                rospy.logwarn("Route show did not stop gracefully, forcing termination")
                self.show_process.kill()
            except Exception as e:
                rospy.logerr(f"Error stopping route show: {e}")
            finally:
                self.show_process = None
        else:
            rospy.loginfo("Route show is not running.")

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    controller = RouteShowController()
    controller.run()