#!/usr/bin/env python3
import rospy
import subprocess
import signal
from std_msgs.msg import String

# Update: Nov 25, 2024

class RouteFollowController:
    def __init__(self):
        rospy.init_node("route_follow_controller", anonymous=True)
        
        self.route_follow_process = None
        
        rospy.Subscriber("/control_route_follow", String, self.control_callback)
        
        rospy.loginfo("Route Follow Controller is running. Waiting for commands...")

    def control_callback(self, msg):
        if msg.data == "start_route_follow":
            self.start_show()
        elif msg.data == "stop_route_follow":
            self.stop_show()
        elif msg.data == "quit":
            rospy.loginfo("Exiting Route Follow Controller.") # this is for debug and test
            if self.route_follow_process is not None:
                self.stop_show()
            rospy.signal_shutdown("User requested shutdown")

    def start_show(self):
        if self.route_follow_process is None:
            rospy.loginfo("Starting route_follow using rosrun...")
            try:
                self.route_follow_process = subprocess.Popen(
                    ["rosrun", "sweep", "route_follow.py"],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE
                )
                rospy.loginfo("Route follow started successfully")

            except Exception as e:
                rospy.logerr(f"Failed to start route show: {e}")
        else:
            rospy.loginfo("Route show is already running.")

    def stop_show(self):
        if self.route_follow_process is not None:
            rospy.loginfo("Stopping route follow...")
            try:
                self.route_follow_process.send_signal(signal.SIGINT)
                self.route_follow_process.wait(timeout=5)
                rospy.loginfo("Route show stopped successfully")
            except subprocess.TimeoutExpired:
                rospy.logwarn("Route show did not stop gracefully, forcing termination")
                self.route_follow_process.kill()
            except Exception as e:
                rospy.logerr(f"Error stopping route show: {e}")
            finally:
                self.route_follow_process = None
        else:
            rospy.loginfo("Route show is not running.")

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    controller = RouteFollowController()
    controller.run()