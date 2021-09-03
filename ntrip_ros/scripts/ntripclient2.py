#!/usr/bin/python

import rospy
from datetime import datetime

# from nmea_msgs.msg import Sentence
from mavros_msgs.msg import RTCM

from base64 import b64encode
from threading import Thread

from httplib import HTTPConnection
from httplib import IncompleteRead

import socket

''' This is to fix the IncompleteRead error
    http://bobrochel.blogspot.com/2010/11/bad-servers-chunked-encoding-and.html'''
import httplib


def patch_http_response_read(func):
    def inner(*args):
        try:
            return func(*args)
        except httplib.IncompleteRead as e:
            return e.partial
    return inner


httplib.HTTPResponse.read = patch_http_response_read(httplib.HTTPResponse.read)


def is_connected(hostname):
    try:
        # see if we can resolve the host name -- tells us if there is
        # a DNS listening
        host = socket.gethostbyname(hostname)
        # connect to the host -- tells us if the host is actually
        # reachable
        s = socket.create_connection((host, 80), 2)
        s.close()
        return True
    except:
        pass
    return False


class ntripclient:
    def __init__(self):
        rospy.init_node('ntripclient', anonymous=True)

        self.rtcm_topic = rospy.get_param('~rtcm_topic', 'rtcm')
        self.nmea_topic = rospy.get_param('~nmea_topic', 'nmea')

        self.ntrip_server = rospy.get_param('~ntrip_server')
        self.ntrip_port = rospy.get_param('~ntrip_port')
        self.ntrip_user = rospy.get_param('~ntrip_user')
        self.ntrip_pass = rospy.get_param('~ntrip_pass')
        self.ntrip_stream = rospy.get_param('~ntrip_stream')
        self.nmea_gga = rospy.get_param('~nmea_gga')

        self.pub = rospy.Publisher(self.rtcm_topic, RTCM, queue_size=10)

        c = is_connected(self.ntrip_server)
        while c is False:
            now = datetime.now()
            current_time = now.strftime("%H:%M:%S")
            print("No internet connection ", current_time)
            c = is_connected(self.ntrip_server)

        print("Active Internet self.connection")
        print(self.ntrip_server+':'+str(self.ntrip_port))

        self.headers = {
            'Ntrip-Version': 'Ntrip/2.0',
            'User-Agent': 'NTRIP ntrip_ros',
            'Connection': 'close',
            'Authorization': 'Basic ' + b64encode(self.ntrip_user + ':' + str(self.ntrip_pass))
        }
        self.connection = HTTPConnection(
            self.ntrip_server+':'+str(self.ntrip_port))
        self.connection.request('GET', '/'+self.ntrip_stream,
                                self.nmea_gga, self.headers)
        self.response = self.connection.getresponse()
        if self.response.status != 200:
            raise Exception("Response.status not 200")
        self.buf = ""
        self.rmsg = RTCM()
        self.restart_count = 0

        rospy.Timer(rospy.Duration(0.1), self.timerCallback)

    def timerCallback(self, event):
        c = is_connected(self.ntrip_server)
        if (c is False):
            now = datetime.now()
            current_time = now.strftime("%H:%M:%S")
            print("No internet connection ", current_time)
            return
        else:
            try:
                data = self.response.read(1)
            except:
                return

        if len(data) != 0:
            if ord(data[0]) == 211:
                self.buf += data
                data = self.response.read(2)
                self.buf += data
                cnt = ord(data[0]) * 256 + ord(data[1])
                data = self.response.read(2)
                self.buf += data
                typ = (ord(data[0]) * 256 + ord(data[1])) / 16
                print(str(datetime.now()), cnt, typ)
                cnt = cnt + 1
                for x in range(cnt):
                    data = self.response.read(1)
                    self.buf += data
                self.rmsg.data = self.buf
                self.rmsg.header.seq += 1
                self.rmsg.header.stamp = rospy.get_rostime()
                self.pub.publish(self.rmsg)
                self.buf = ""
            else:
                print(data)
        else:
            ''' If zero length data, close self.connection and reopen it '''
            self.restart_count = self.restart_count + 1
            print("Zero length ", self.restart_count)
            self.connection.close()

            c = is_connected(self.ntrip_server)
            while c is False:
                print("Waiting for active internet self.connection")
                c = is_connected(self.ntrip_server)

            self.connection = HTTPConnection(self.ntrip_server)
            self.connection.request(
                'GET', '/'+self.ntrip_stream, self.nmea_gga, self.headers)
            self.response = self.connection.getresponse()
            if self.response.status != 200:
                raise Exception("Response.status not 200")
            self.buf = ""


if __name__ == '__main__':
    c = ntripclient()
    rospy.spin()
