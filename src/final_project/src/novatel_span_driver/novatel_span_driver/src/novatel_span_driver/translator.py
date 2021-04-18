#! /usr/bin/env python
# -*- coding: utf-8 -*-

# Software License Agreement (BSD)
#
#  file      @translator.py
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

import roslib
import roslib.message
import roslib.msgs

import genpy
import rospy
import struct
from itertools import izip
from cStringIO import StringIO


class EndOfBuffer(BaseException):
    pass


class TranslatorError(ValueError):
    pass


class Handler(object):
    def field(self, msg):
        return getattr(msg, self.name)

    def preserialize(self, msg):
        pass


class SubMessageHandler(Handler):
    def __init__(self, field):
        self.name = field.name
        self.msg_cls = roslib.message.get_message_class(field.type)

    def deserialize(self, buff, msg):
        self.field(msg).translator().deserialize(buff)

    def serialize(self, buff, msg):
        self.field(msg).translator().serialize(buff)


class FixedFieldsHandler(Handler):
    def __init__(self, fields):
        struct_strs = ['<']

        # serial vs ethernet
        def pattern(field):
            try:
                return genpy.base.SIMPLE_TYPES_DICT[field.type]
            except KeyError:
                if field.base_type in ['uint8', 'char'] and field.array_len is not None:
                    return "%is" % field.array_len
                else:
                    raise

        struct_strs.extend([pattern(f) for f in fields])
        self.struct = struct.Struct(''.join(struct_strs))
        self.names = [f.name for f in fields]
        self.size = self.struct.size

    def serialize(self, buff, msg):
        buff.write(self.struct.pack(*[getattr(msg, name) for name in self.names]))

    def deserialize(self, buff, msg):
        st = buff.read(self.struct.size)
        if st == '':
            return
        values = self.struct.unpack(st)
        for name, value in izip(self.names, values):
            setattr(msg, name, value)


class SubMessageArrayHandler(Handler):
    struct_uint16 = struct.Struct('<H')
    struct_uint8 = struct.Struct('<B')

    def __init__(self, field):
        self.name = field.name
        self.name_count = "%s_count" % self.name
        self.msg_cls = roslib.message.get_message_class(field.base_type)
        self.submessage_size = self.msg_cls().translator().size

    def deserialize(self, buff, msg):
        if hasattr(msg, self.name_count):
            # Another field specifies number of array items to deserialize.
            length = getattr(msg, self.name_count) * self.submessage_size
            data = StringIO(buff.read(length))
        else:
            # Consume as much as we can straight from the buffer.
            data = buff

        # Find and empty the array to be populated.
        array = self.field(msg)
        array[:] = []

        try:
            while True:
                submessage = self.msg_cls()
                submessage.translator().deserialize(data)
                array.append(submessage)
        except EndOfBuffer:
            pass

    def serialize(self, buff, msg):
        for submessage in self.field(msg):
            submessage.translator().serialize(buff)

    def preserialize(self, msg):
        if hasattr(msg, self.name_count):
            setattr(msg, self.name_count, len(self.field(msg)))


class VariableStringHandler(Handler):
    struct_bytes = struct.Struct('<H')

    def __init__(self, field):
        self.name = field.name

    def deserialize(self, buff, msg):
        length = self.struct_bytes.unpack(buff.read(self.struct_bytes.size))[0]
        setattr(msg, self.name, str(buff.read(length)))


class Translator:
    def __init__(self, msg_cls):
        self.handlers = []
        self.size = None

        cls_name, spec = roslib.msgs.load_by_type(msg_cls._type)

        fixed_fields = []
        for field in spec.parsed_fields():
            if field.type == 'novatel_msgs/CommonHeader':
                # Deserializing the header happens elsewhere.
                continue

            if genpy.base.is_simple(field.base_type) and (field.array_len is not None or not field.is_array):
                # Simple types and fixed-length character arrays.
                fixed_fields.append(field)
            else:
                # Before dealing with this non-simple field, add a handler for the fixed fields
                # encountered so far.
                if len(fixed_fields) > 0:
                    self.handlers.append(FixedFieldsHandler(fixed_fields))
                    fixed_fields = []

                # Handle this other type.
                if field.type == 'string' or (field.base_type == 'uint8' and field.is_array):
                    self.handlers.append(VariableStringHandler(field))
                elif field.is_array:
                    self.handlers.append(SubMessageArrayHandler(field))
                else:
                    self.handlers.append(SubMessageHandler(field))

        if len(fixed_fields) > 0:
            self.handlers.append(FixedFieldsHandler(fixed_fields))

        if len(self.handlers) == 1 and hasattr(self.handlers[0], 'size'):
            self.size = self.handlers[0].size


class TranslatorProxy:
    def __init__(self, translator, msg):
        self.translator = translator
        self.size = translator.size
        self.msg = msg

    def deserialize(self, buff):
        try:
            for handler in self.translator.handlers:
                handler.deserialize(buff, self.msg)
        except struct.error as e:
            raise TranslatorError(e)

    def serialize(self, buff):
        try:
            for handler in self.translator.handlers:
                handler.serialize(buff, self.msg)
        except struct.error as e:
            raise TranslatorError(e)

    def preserialize(self):
        for handler in self.translator.handlers:
            handler.preserialize(self.msg)


def translator(self):
    if not hasattr(self.__class__, "_translator"):
        self.__class__._translator = Translator(self.__class__)
    return TranslatorProxy(self.__class__._translator, self)

roslib.message.Message.translator = translator
