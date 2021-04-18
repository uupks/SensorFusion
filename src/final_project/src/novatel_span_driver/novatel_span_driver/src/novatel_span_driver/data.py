#! /usr/bin/env python
# -*- coding: utf-8 -*-

# Software License Agreement (BSD)
#
#  file      @data.py
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

import rospy
import novatel_msgs.msg

from port import Port
from novatel_span_driver.mapping import msgs
from handlers import MessageHandler
import translator

from cStringIO import StringIO
from threading import Lock


class DataPort(Port):
    def run(self):
        # Set up handlers for translating different novatel messages as they arrive.
        handlers = {}
        pkt_counters = {}
        pkt_times = {}

        for msg_id in msgs.keys():
            handlers[msg_id] = MessageHandler(*msgs[msg_id])
            pkt_counters[msg_id] = 0
            pkt_times[msg_id] = 0

        bad_pkts = set()
        pkt_id = None

        while not self.finish.is_set():
            try:
                header, pkt_str = self.recv()
                if header is not None:
                    handlers[header.id].handle(StringIO(pkt_str), header)

            except ValueError as e:
                # Some problem in the recv() routine.
                rospy.logwarn(str(e))
                continue

            except KeyError as e:
                if header.id not in handlers and header.id not in pkt_counters:
                    rospy.logwarn("No handler for message id %d" % header.id)

            except translator.TranslatorError:
                if header.id not in bad_pkts:
                    rospy.logwarn("Error parsing %s.%d" % header.id)
                    bad_pkts.add(pkt)

            if header.id not in pkt_counters:
                pkt_counters[header.id] = 0
            else:
                pkt_counters[header.id] += 1
                pkt_times[header.id] = header.gps_week_seconds  # only track times of msgs that are part of novatel msgs
