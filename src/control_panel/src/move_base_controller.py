#!/usr/bin/env python3

import rospy
import subprocess
import signal
from std_msgs.msg import String

# Latest Update: Nov 25, 2024

class MoveBaseController:
    def __init__(self):
        rospy.init_node("move_base_controller", anonymous=True)
        self.move_base_process = None
        rospy.Subscriber("/control_move_base", String, self.control_callback)
        rospy.loginfo("Move Base Controller is running. Waiting for commands...")

    def control_callback(self, msg):
        # Check the command from the control panel
        if msg.data == "start_move_base":
            self.start_move_base()
        elif msg.data == "stop_move_base":
            self.stop_move_base()
        elif msg.data == "quit":
            rospy.loginfo("Exiting Move Base Controlstop_move_baseler.")
            if self.move_base_process is not None:
                self.stop_move_base()  # Ensure move_base is stopped before quitting
            rospy.signal_shutdown("User requested shutdown")

    def start_move_base(self):
        if self.move_base_process is None:
            rospy.loginfo("Starting move_base.launch...")
            # Start move_base.launch as a subprocess
            self.move_base_process = subprocess.Popen(["roslaunch", "turtlebot3_navigation", "move_base.launch"])
        else:
            rospy.loginfo("move_base is already running.")

    def stop_move_base(self):
        if self.move_base_process is not None:
            rospy.loginfo("Stopping move_base.launch...")
            # Send SIGINT to terminate the launch process (similar to pressing Ctrl+C)
            self.move_base_process.send_signal(signal.SIGINT)
            self.move_base_process = None
        else:
            rospy.loginfo("move_base is not running.")

    def run(self):
        rospy.spin()  # Keep the node running to listen for commands

if __name__ == "__main__":
    controller = MoveBaseController()
    controller.run()
