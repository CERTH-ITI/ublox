#!/usr/bin/python

import rospy
from std_srvs.srv import Empty
import os

def killRequestCB(req):
  os.system("rosnode kill ntrip_ros")
  rospy.loginfo("Killed ntrip")


def kill_client_server():
    rospy.init_node('restart_ntrip_node')
    s = rospy.Service('kill_ntrip', Empty, killRequestCB)
    rospy.spin()

if __name__ == "__main__":
    kill_client_server()