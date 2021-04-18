#! /usr/bin/env python
# -*- coding: utf-8 -*-

# Software License Agreement (BSD)
#
#  file      @bridge.py
#  authors   Mike Purvis <mpurvis@clearpathrobotics.com>
#            NovAtel <novatel.com/support>
#  copyright Copyright (c) 2012, Clearpath Robotics, Inc., All rights reserved.
#            Copyright (c) 2014, NovAtel Inc., All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that
# the following conditions are met:
#  * Redistributions of source code must retain the above copyright notice, this list of conditions and the
#    following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
#    following disclaimer in the documentation and/or other materials provided with the distribution.
#  * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote
#    products derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
# RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
# PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
# DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
# OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# ROS
import rospy
from novatel_msgs.msg import *
from std_msgs.msg import String

# Package modules
from novatel_span_driver.data import DataPort
from novatel_span_driver.monitor import Monitor

# Standard
import socket
import serial
import struct
from cStringIO import StringIO
import time

from novatel_span_driver import translator

DEFAULT_IP = '198.161.73.9'
DEFAULT_PORT = 3001
DEFAULT_DEV_NO = '/dev/ttyUSB0'
DEFAULT_BAUDRATE = 115200

SOCKET_TIMEOUT = 100.0
socks = []
ports = {}
monitor = Monitor(ports)


def init():
    # Pass this parameter to use pcap data rather than a socket to a device.
    # For testing the node itself--to exercise downstream algorithms, use a bag.
    pcap_file_name = rospy.get_param('~pcap_file', False)

    if not pcap_file_name:
        connect_type = rospy.get_param('~connect_type', False)
        if not connect_type:
            rospy.logfatal("Failed to fetch parameter: connect_type, please configure it as 'TCP' or 'SERIAL'")
            exit(1)

        sock = create_sock('data', connect_type)
    else:
        sock = create_test_sock(pcap_file_name)

    ports['data'] = DataPort(sock)

    configure_receiver(sock)

    for name, port in ports.items():
        port.start()
        rospy.loginfo("Port %s thread started." % name)
    monitor.start()

    rospy.on_shutdown(shutdown)


def create_sock(name, connect_type):
    try:
        if "TCP" == connect_type:
            ip = rospy.get_param('~ip', DEFAULT_IP)
            port = rospy.get_param('~port', DEFAULT_PORT)
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            ip_port = (ip, port)
            sock.connect(ip_port)
            rospy.loginfo("Successfully connected to %%s port at %s:%d" % ip_port % name)
        elif "SERIAL" == connect_type:
            dev_no = rospy.get_param('~dev_no', DEFAULT_DEV_NO)
            baud = rospy.get_param('~baudrate', DEFAULT_BAUDRATE)
            sock = serial.Serial(port=dev_no, baudrate=baud, timeout=SOCKET_TIMEOUT, rtscts=True, dsrdtr=True)
            rospy.loginfo("Successfully connected to %%s port at %s:%d" % (dev_no, baud) % name)

            # make methods dynamically for make serial obj be compatible with socket obj
            from types import MethodType
            sock.recv = MethodType(serial.Serial.read, sock, serial.Serial)
            sock.send = MethodType(serial.Serial.write, sock, serial.Serial)
            sock.settimeout = MethodType(lambda *args, **kwargs: None, sock, serial.Serial)
            sock.shutdown = MethodType(lambda *args, **kwargs: None, sock, serial.Serial)
        else:
            rospy.logfatal("Connect type: %s isn't supported yet, please configure it as 'TCP' or 'SERIAL'"
                           % connect_type)
            exit(1)
    except (socket.error, serial.SerialException) as e:
        rospy.logfatal("Couldn't connect to port at %s:%s" % (name, str(e)))
        exit(1)
    sock.settimeout(SOCKET_TIMEOUT)
    socks.append(sock)
    return sock


def create_test_sock(pcap_filename):
    rospy.sleep(0.1)

    try:
        import pcapy
    except ImportError:
        import pure_pcapy as pcapy

    from StringIO import StringIO
    from impacket import ImpactDecoder

    body_list = []
    if pcap_filename.endswith("gz"):
        # From: http://projects.honeynet.org/honeysnap/changeset/35/trunk/honeysnap/__init__.py
        import tempfile
        import gzip
        tmph, tmpf = tempfile.mkstemp()
        tmph = open(tmpf, 'wb')
        gfile = gzip.open(pcap_filename)
        tmph.write(gfile.read())
        gfile.close()
        tmph.close()
        pcap_filename = tmpf

    cap = pcapy.open_offline(pcap_filename)
    decoder = ImpactDecoder.EthDecoder()

    while True:
        header, payload = cap.next()
        if not header:
            break
        try:
            tcp = decoder.decode(payload).child().child()
            body_list.append(tcp.child().get_packet())
        except AttributeError:
            print(decoder.decode(payload))
            raise

    data_io = StringIO(''.join(body_list))

    class MockSocket(object):

        def recv(self, byte_count):
            rospy.sleep(0.002)
            data = data_io.read(byte_count)
            if data == "":
                rospy.signal_shutdown("Test completed.")
            return data

        def settimeout(self, timeout):
            pass

    return MockSocket()


def configure_receiver(port):
    receiver_config = rospy.get_param('~configuration', None)

    if receiver_config is not None:
        imu_connect = receiver_config.get('imu_connect', None)
        if imu_connect is not None:
            rospy.loginfo("Sending IMU connection string to SPAN system.")
            port.send('connectimu ' + imu_connect['port'] + ' ' + imu_connect['type'] + '\r\n')

        logger = receiver_config.get('log_request', [])
        rospy.loginfo("Enabling %i log outputs from SPAN system." % len(logger))
        for log in logger:
            port.send('log ' + log + ' ontime ' + str(logger[log]) + '\r\n')

        commands = receiver_config.get('command', [])
        rospy.loginfo("Sending %i user-specified initialization commands to SPAN system." % len(commands))
        for cmd in commands:
            port.send(cmd + ' ' + str(commands[cmd]) + '\r\n')


def shutdown():
    monitor.finish.set()
    monitor.join()
    rospy.loginfo("Thread monitor finished.")
    for name, port in ports.items():
        port.finish.set()
        port.join()
        rospy.loginfo("Port %s thread finished." % name)
    for sock in socks:
        sock.shutdown(socket.SHUT_RDWR)
        sock.close()
    rospy.loginfo("Sockets closed.")
