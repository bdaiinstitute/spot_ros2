# generated from rosidl_generator_py/resource/_idl.py.em
# with input from spot_msgs:msg/BehaviorFault.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_BehaviorFault(type):
    """Metaclass of message 'BehaviorFault'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'CAUSE_UNKNOWN': 0,
        'CAUSE_FALL': 1,
        'CAUSE_HARDWARE': 2,
        'STATUS_UNKNOWN': 0,
        'STATUS_CLEARABLE': 1,
        'STATUS_UNCLEARABLE': 2,
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
                'spot_msgs.msg.BehaviorFault')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__behavior_fault
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__behavior_fault
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__behavior_fault
            cls._TYPE_SUPPORT = module.type_support_msg__msg__behavior_fault
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__behavior_fault

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'CAUSE_UNKNOWN': cls.__constants['CAUSE_UNKNOWN'],
            'CAUSE_FALL': cls.__constants['CAUSE_FALL'],
            'CAUSE_HARDWARE': cls.__constants['CAUSE_HARDWARE'],
            'STATUS_UNKNOWN': cls.__constants['STATUS_UNKNOWN'],
            'STATUS_CLEARABLE': cls.__constants['STATUS_CLEARABLE'],
            'STATUS_UNCLEARABLE': cls.__constants['STATUS_UNCLEARABLE'],
        }

    @property
    def CAUSE_UNKNOWN(self):
        """Message constant 'CAUSE_UNKNOWN'."""
        return Metaclass_BehaviorFault.__constants['CAUSE_UNKNOWN']

    @property
    def CAUSE_FALL(self):
        """Message constant 'CAUSE_FALL'."""
        return Metaclass_BehaviorFault.__constants['CAUSE_FALL']

    @property
    def CAUSE_HARDWARE(self):
        """Message constant 'CAUSE_HARDWARE'."""
        return Metaclass_BehaviorFault.__constants['CAUSE_HARDWARE']

    @property
    def STATUS_UNKNOWN(self):
        """Message constant 'STATUS_UNKNOWN'."""
        return Metaclass_BehaviorFault.__constants['STATUS_UNKNOWN']

    @property
    def STATUS_CLEARABLE(self):
        """Message constant 'STATUS_CLEARABLE'."""
        return Metaclass_BehaviorFault.__constants['STATUS_CLEARABLE']

    @property
    def STATUS_UNCLEARABLE(self):
        """Message constant 'STATUS_UNCLEARABLE'."""
        return Metaclass_BehaviorFault.__constants['STATUS_UNCLEARABLE']


class BehaviorFault(metaclass=Metaclass_BehaviorFault):
    """
    Message class 'BehaviorFault'.

    Constants:
      CAUSE_UNKNOWN
      CAUSE_FALL
      CAUSE_HARDWARE
      STATUS_UNKNOWN
      STATUS_CLEARABLE
      STATUS_UNCLEARABLE
    """

    __slots__ = [
        '_header',
        '_behavior_fault_id',
        '_cause',
        '_status',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'behavior_fault_id': 'uint32',
        'cause': 'uint8',
        'status': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.behavior_fault_id = kwargs.get('behavior_fault_id', int())
        self.cause = kwargs.get('cause', int())
        self.status = kwargs.get('status', int())

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
        if self.header != other.header:
            return False
        if self.behavior_fault_id != other.behavior_fault_id:
            return False
        if self.cause != other.cause:
            return False
        if self.status != other.status:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if __debug__:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @builtins.property
    def behavior_fault_id(self):
        """Message field 'behavior_fault_id'."""
        return self._behavior_fault_id

    @behavior_fault_id.setter
    def behavior_fault_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'behavior_fault_id' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'behavior_fault_id' field must be an unsigned integer in [0, 4294967295]"
        self._behavior_fault_id = value

    @builtins.property
    def cause(self):
        """Message field 'cause'."""
        return self._cause

    @cause.setter
    def cause(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'cause' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'cause' field must be an unsigned integer in [0, 255]"
        self._cause = value

    @builtins.property
    def status(self):
        """Message field 'status'."""
        return self._status

    @status.setter
    def status(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'status' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'status' field must be an unsigned integer in [0, 255]"
        self._status = value
