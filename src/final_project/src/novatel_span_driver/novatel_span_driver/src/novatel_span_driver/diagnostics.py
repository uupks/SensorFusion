#! /usr/bin/env python
# -*- coding: utf-8 -*-

# Software License Agreement (BSD)
#
#  file      @publisher.py
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
import diagnostic_updater

from diagnostic_msgs.msg import DiagnosticStatus
from novatel_msgs.msg import BESTPOS, INSPVAX


class NovatelDiagnostics(object):
    def __init__(self):
        self.last_bestpos = None
        self.last_inspvax = None
        rospy.Subscriber("novatel_data/bestpos", BESTPOS, self.bestpos_callback)
        rospy.Subscriber("novatel_data/inspvax", INSPVAX, self.inspvax_callback)

        self.updater = diagnostic_updater.Updater()
        self.updater.setHardwareID("none")
        self.updater.add("Novatel SPAN", self.produce_diagnostics)

    def bestpos_callback(self, msg):
        self.last_bestpos = msg
        self.updater.setHardwareID("firmware-%d" % msg.header.software_version)
        self.updater.update()

    def inspvax_callback(self, msg):
        self.last_inspvax = msg
        self.updater.update()

    @staticmethod
    def get_status_string(msg, field):
        value = getattr(msg, field)
        matching_status = [x[len(field) + 1:] for x in dir(msg) if x.startswith(field.upper()) and
                           value == getattr(msg, x)]
        if len(matching_status) != 1:
            return "No matching constant"
        return matching_status[0]

    @staticmethod
    def get_status_bitfield(msg, field):
        value = getattr(msg, field)
        matching_statuses = [x[len(field) + 1:] for x in dir(msg) if x.startswith(field.upper()) and
                             value & getattr(msg, x)]
        return ', '.join(matching_statuses)

    def produce_diagnostics(self, stat):
        if self.last_bestpos:
            stat.add("GNSS Solution Status",
                     self.get_status_string(self.last_bestpos, "solution_status"))
            stat.add("GNSS Position Type",
                     self.get_status_string(self.last_bestpos, "position_type"))
            self.last_bestpos = None

        if self.last_inspvax:
            if self.last_inspvax.ins_status != INSPVAX.INS_STATUS_SOLUTION_GOOD:
                stat.summary(DiagnosticStatus.WARN, "INS Solution not GOOD.")
            elif self.last_inspvax.position_type != INSPVAX.POSITION_TYPE_PPP:
                stat.summary(DiagnosticStatus.WARN, "INS Position type not PPP.")
            else:
                stat.summary(DiagnosticStatus.OK, "INS Solution GOOD, PPP fix present.")

            stat.add("INS Solution Status",
                     self.get_status_string(self.last_inspvax, "ins_status"))
            stat.add("INS Position Type",
                     self.get_status_string(self.last_inspvax, "position_type"))
            stat.add("INS Extended Status",
                     self.get_status_bitfield(self.last_inspvax, "extended_status"))
            stat.add("Seconds since last ZUPT or position update.",
                     self.last_inspvax.seconds_since_update)
            stat.add("Receiver Status",
                     self.get_status_bitfield(self.last_inspvax.header, "receiver_status"))
            self.last_inspvax = None
        else:
            stat.summary(DiagnosticStatus.ERROR,
                         "No INSPVAX logs received from Novatel system.")

        return stat
