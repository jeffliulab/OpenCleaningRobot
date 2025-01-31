#!/usr/bin/env python3
import rospy
import subprocess
import signal
import os
from std_msgs.msg import String

# Update: Nov25, add state publish into this file, to debug of SLAM

# UPDATE: Nov22, 2024
# Use sweep/route_map_server to control AMCL, because AMCL needs map_server
# Also use RVIZ to see the work

class RouteController:
   def __init__(self):
       rospy.init_node("route_controller", anonymous=True)
       self.route_process = None
       self.robot_state_pub = None
       rospy.Subscriber("/control_amcl", String, self.control_callback)
       rospy.loginfo("Route Controller is running. Waiting for commands...")

   def control_callback(self, msg):
       if msg.data == "start_amcl":
           self.start_route()
       #elif msg.data == "start_amcl_v2":
       #    self.start_route_v2()
       elif msg.data == "stop_amcl":
           self.stop_route()
       elif msg.data == "quit":
           rospy.loginfo("Exiting Route Controller.")
           if self.route_process is not None:
               self.stop_route()
           rospy.signal_shutdown("User requested shutdown")

   def start_route(self):
       if self.route_process is None:
           rospy.loginfo("Starting route_map_server using rosrun...")
           try:
               self.robot_state_pub = subprocess.Popen(["rosrun", "robot_state_publisher", "robot_state_publisher"])
               self.route_process = subprocess.Popen(
                   ["rosrun", "sweep", "route_map_server.py"],
                   stdout=subprocess.PIPE,
                   stderr=subprocess.PIPE
               )
               rospy.loginfo("Route map server started successfully")
           except Exception as e:
               rospy.logerr(f"Failed to start route map server: {e}")
       else:
           rospy.loginfo("Route map server is already running.")

   #def start_route_v2(self):
   #    if self.route_process is None:
   #        rospy.loginfo("Starting route_map_server using rosrun...")
   #        try:
   #            self.robot_state_pub = subprocess.Popen(["rosrun", "robot_state_publisher", "robot_state_publisher"])
   #            self.route_process = subprocess.Popen(
   #                ["rosrun", "sweep", "route_map_server_v2.py"],
   #                stdout=subprocess.PIPE,
   #                stderr=subprocess.PIPE
   #            )
   #            rospy.loginfo("Route map server started successfully")
   #        except Exception as e:
   #            rospy.logerr(f"Failed to start route map server: {e}")
   #    else:
   #        rospy.loginfo("Route map server is already running.")


   def stop_route(self):
       if self.route_process is not None:
           rospy.loginfo("Stopping route map server...")
           try:
               self.route_process.send_signal(signal.SIGINT)
               self.robot_state_pub.send_signal(signal.SIGINT)
               self.route_process.wait(timeout=5)
               rospy.loginfo("Route map server and robot state publisher stopped successfully")
           except subprocess.TimeoutExpired:
               rospy.logwarn("Processes did not stop gracefully, forcing termination")
               self.route_process.kill()
               self.robot_state_pub.kill()
           except Exception as e:
               rospy.logerr(f"Error stopping processes: {e}")
           finally:
               self.route_process = None
               self.robot_state_pub = None
       else:
           rospy.loginfo("Route map server is not running.")

   def run(self):
       rospy.spin()

if __name__ == "__main__":
   controller = RouteController()
   controller.run()