#!/usr/bin/python

import rospy
from datetime import datetime

#from nmea_msgs.msg import Sentence
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
        except httplib.IncompleteRead, e:
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


class ntripconnect(Thread):
    def __init__(self, ntc):
        super(ntripconnect, self).__init__()
        self.ntc = ntc
        self.stop = False

    def run(self):

        headers = {
            'Ntrip-Version': 'Ntrip/2.0',
            'User-Agent': 'NTRIP ntrip_ros',
            'Connection': 'close',
            'Authorization': 'Basic ' + b64encode(self.ntc.ntrip_user + ':' + str(self.ntc.ntrip_pass))
        }
        connection = HTTPConnection(self.ntc.ntrip_server+':'+str(self.ntc.ntrip_port))
        connection.request('GET', '/'+self.ntc.ntrip_stream,
                           self.ntc.nmea_gga, headers)
        response = connection.getresponse()
        if response.status != 200:
            raise Exception("Response.status not 200")
        buf = ""
        rmsg = RTCM()
        restart_count = 0
        while not self.stop:
            '''
            data = response.read(100)
            pos = data.find('\r\n')
            if pos != -1:
                rmsg.message = buf + data[:pos]
                rmsg.header.seq += 1
                rmsg.header.stamp = rospy.get_rostime()
                buf = data[pos+2:]
                self.ntc.pub.publish(rmsg)
            else: buf += data
            '''

            ''' This now separates individual RTCM messages and publishes each one on the same topic '''
            c = is_connected(self.ntc.ntrip_server)
            if (c is False):
                print("Not connected\n")
                continue
            else:
                try:
                    data = response.read(1)
                except:
                    print("Cannot read\n")
                    continue                
                    
            if len(data) != 0:
                if ord(data[0]) == 211:
                    buf += data
                    data = response.read(2)
                    buf += data
                    cnt = ord(data[0]) * 256 + ord(data[1])
                    data = response.read(2)
                    buf += data
                    typ = (ord(data[0]) * 256 + ord(data[1])) / 16
                    print(str(datetime.now()), cnt, typ)
                    cnt = cnt + 1
                    for x in range(cnt):
                        data = response.read(1)
                        buf += data
                    rmsg.data = buf
                    rmsg.header.seq += 1
                    rmsg.header.stamp = rospy.get_rostime()
                    self.ntc.pub.publish(rmsg)
                    buf = ""
                else:
                    print(data)
            else:
                ''' If zero length data, close connection and reopen it '''
                restart_count = restart_count + 1
                print("Zero length ", restart_count)
                connection.close()

                c = is_connected(self.ntrip_server)
                while c is False:
                    print("Waiting for active internet connection")
                    c = is_connected(self.ntrip_server)

                connection = HTTPConnection(self.ntc.ntrip_server)
                connection.request(
                    'GET', '/'+self.ntc.ntrip_stream, self.ntc.nmea_gga, headers)
                response = connection.getresponse()
                if response.status != 200:
                    raise Exception("Response.status not 200")
                buf = ""

        connection.close()


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
            print("Waiting for active internet connection")
            c = is_connected(self.ntrip_server)

        # self.ntrip_server = self.ntrip_server+':'+str(self.ntrip_port)
        print("Connected!")
        print(self.ntrip_server+':'+str(self.ntrip_port))

        self.connection = None
        self.connection = ntripconnect(self)
        self.connection.start()

    def run(self):
        rospy.spin()
        if self.connection is not None:
            self.connection.stop = True


if __name__ == '__main__':
    c = ntripclient()
    c.run()
