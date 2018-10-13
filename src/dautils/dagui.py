#!/usr/bin/env python
"""
DaGui
======
This is just a basic graphical interface
to test out the robot actions
"""


import rospy
import actionlib
from dabot.msg import CalibrateAction, CalibrateGoal
from Tkinter import Tk, Label, Button


class DaGui:
    def __init__(self, master):
        self.master = master
        master.title("Design Assistant Robot Control GUI")
        rospy.init_node("DaGui", anonymous=True)
        self.define_buttons()
        self.init_publishers()
        self.init_services()
        self.init_action_clients()
        self.init_subscribers()

    def init_publishers(self):
        pass

    def init_services(self):
        pass

    def init_action_clients(self):
        self.calibration_client = actionlib.SimpleActionClient("calibrate_arm", CalibrateAction)

    def init_subscribers(self):
        pass

    def call_calibrate(self):
        self.calibration_client.wait_for_server(timeout=rospy.Duration(5))
        goal = CalibrateGoal("calibrate_arm")
        self.calibration_client.send_goal(goal)
        self.calibration_client.wait_for_result()
        return self.calibration_client.get_result()

    def define_buttons(self):
        self.label = Label(self.master, text="Calibrate the robot")
        self.label.pack()
        self.greet_button = Button(self.master, text="Calibrate", command=self.call_calibrate)
        self.greet_button.pack()

        self.label2 = Label(self.master, text="Control the mofo2")
        self.label2.pack()
        self.greet_button = Button(self.master, text="Gree2t", command=self.greet)
        self.greet_button.pack()

        self.close_button = Button(self.master, text="Close", command=self.master.quit)
        self.close_button.pack()

    def greet(self):
        print("Greetings!")


if __name__ == "__main__":
    root = Tk()
    my_gui = DaGui(root)
    root.mainloop()
