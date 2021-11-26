#!/usr/bin/python

import rospy
from std_srvs.srv import Empty
import os

bus = '001'
device = '008'


def killRequestCB(req):
  os.system("rosnode kill ntrip_ros")
  rospy.loginfo("Killed ntrip")

def resetRequestCB(req):
  rospy.loginfo("RESETTING GPS USB")
  os.system("rosrun ntrip_ros usbreset /dev/bus/usb/"+bus+"/"+device)
  rospy.loginfo("Killed ntrip")


def kill_client_server():
    rospy.init_node('restart_ntrip_node')
    s = rospy.Service('kill_ntrip', Empty, killRequestCB)
    s_reset = rospy.Service('reset_usb', Empty, resetRequestCB)
    bus = rospy.get_param('~bus_id')
    device = rospy.get_param('~device_id')
    print('bus: '+bus+' device: '+device)
    rospy.spin()

if __name__ == "__main__":
    kill_client_server()