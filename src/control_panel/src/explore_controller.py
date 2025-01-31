#!/usr/bin/env python3

import rospy
import subprocess
import signal
from std_msgs.msg import String

# Latest Update: Nov 25, 2024

class ExploreController:
    def __init__(self):
        rospy.init_node("explore_controller", anonymous=True)
        self.explore_process = None
        rospy.Subscriber("/control_explore", String, self.control_callback)
        rospy.loginfo("Explore Controller is running. Waiting for commands...")

    def control_callback(self, msg):
        # Check the command from the control panel
        if msg.data == "start_explore":
            self.start_explore()
        elif msg.data == "stop_explore":
            self.stop_explore()
        elif msg.data == "quit":
            rospy.loginfo("Exiting Explore Controller.")
            if self.explore_process is not None:
                self.stop_explore()  # Ensure explore is stopped before quitting
            rospy.signal_shutdown("User requested shutdown")

    def start_explore(self):
        if self.explore_process is None:
            rospy.loginfo("Starting explore.launch...")
            # Start explore.launch as a subprocess
            self.explore_process = subprocess.Popen(["roslaunch", "explore_lite", "explore.launch"])
        else:
            rospy.loginfo("Explore is already running.")

    def stop_explore(self):
        if self.explore_process is not None:
            rospy.loginfo("Stopping explore.launch...")
            # Send SIGINT to terminate the launch process (similar to pressing Ctrl+C)
            self.explore_process.send_signal(signal.SIGINT)
            self.explore_process = None
        else:
            rospy.loginfo("Explore is not running.")

    def run(self):
        rospy.spin()  # Keep the node running to listen for commands

if __name__ == "__main__":
    controller = ExploreController()
    controller.run()
