# generated from rosidl_generator_py/resource/_idl.py.em
# with input from spot_msgs:msg/Lease.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'sequence'
import array  # noqa: E402, I100

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Lease(type):
    """Metaclass of message 'Lease'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('spot_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'spot_msgs.msg.Lease')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__lease
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__lease
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__lease
            cls._TYPE_SUPPORT = module.type_support_msg__msg__lease
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__lease

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Lease(metaclass=Metaclass_Lease):
    """Message class 'Lease'."""

    __slots__ = [
        '_resource',
        '_epoch',
        '_sequence',
    ]

    _fields_and_field_types = {
        'resource': 'string',
        'epoch': 'string',
        'sequence': 'sequence<uint32>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('uint32')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.resource = kwargs.get('resource', str())
        self.epoch = kwargs.get('epoch', str())
        self.sequence = array.array('I', kwargs.get('sequence', []))

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.resource != other.resource:
            return False
        if self.epoch != other.epoch:
            return False
        if self.sequence != other.sequence:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def resource(self):
        """Message field 'resource'."""
        return self._resource

    @resource.setter
    def resource(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'resource' field must be of type 'str'"
        self._resource = value

    @builtins.property
    def epoch(self):
        """Message field 'epoch'."""
        return self._epoch

    @epoch.setter
    def epoch(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'epoch' field must be of type 'str'"
        self._epoch = value

    @builtins.property
    def sequence(self):
        """Message field 'sequence'."""
        return self._sequence

    @sequence.setter
    def sequence(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'I', \
                "The 'sequence' array.array() must have the type code of 'I'"
            self._sequence = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, int) for v in value) and
                 all(val >= 0 and val < 4294967296 for val in value)), \
                "The 'sequence' field must be a set or sequence and each value of type 'int' and each unsigned integer in [0, 4294967295]"
        self._sequence = array.array('I', value)
