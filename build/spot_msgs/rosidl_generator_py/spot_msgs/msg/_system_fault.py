# generated from rosidl_generator_py/resource/_idl.py.em
# with input from spot_msgs:msg/SystemFault.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_SystemFault(type):
    """Metaclass of message 'SystemFault'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'SEVERITY_UNKNOWN': 0,
        'SEVERITY_INFO': 1,
        'SEVERITY_WARN': 2,
        'SEVERITY_CRITICAL': 3,
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
                'spot_msgs.msg.SystemFault')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__system_fault
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__system_fault
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__system_fault
            cls._TYPE_SUPPORT = module.type_support_msg__msg__system_fault
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__system_fault

            from builtin_interfaces.msg import Duration
            if Duration.__class__._TYPE_SUPPORT is None:
                Duration.__class__.__import_type_support__()

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'SEVERITY_UNKNOWN': cls.__constants['SEVERITY_UNKNOWN'],
            'SEVERITY_INFO': cls.__constants['SEVERITY_INFO'],
            'SEVERITY_WARN': cls.__constants['SEVERITY_WARN'],
            'SEVERITY_CRITICAL': cls.__constants['SEVERITY_CRITICAL'],
        }

    @property
    def SEVERITY_UNKNOWN(self):
        """Message constant 'SEVERITY_UNKNOWN'."""
        return Metaclass_SystemFault.__constants['SEVERITY_UNKNOWN']

    @property
    def SEVERITY_INFO(self):
        """Message constant 'SEVERITY_INFO'."""
        return Metaclass_SystemFault.__constants['SEVERITY_INFO']

    @property
    def SEVERITY_WARN(self):
        """Message constant 'SEVERITY_WARN'."""
        return Metaclass_SystemFault.__constants['SEVERITY_WARN']

    @property
    def SEVERITY_CRITICAL(self):
        """Message constant 'SEVERITY_CRITICAL'."""
        return Metaclass_SystemFault.__constants['SEVERITY_CRITICAL']


class SystemFault(metaclass=Metaclass_SystemFault):
    """
    Message class 'SystemFault'.

    Constants:
      SEVERITY_UNKNOWN
      SEVERITY_INFO
      SEVERITY_WARN
      SEVERITY_CRITICAL
    """

    __slots__ = [
        '_header',
        '_name',
        '_duration',
        '_code',
        '_uid',
        '_error_message',
        '_attributes',
        '_severity',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'name': 'string',
        'duration': 'builtin_interfaces/Duration',
        'code': 'int32',
        'uid': 'uint64',
        'error_message': 'string',
        'attributes': 'sequence<string>',
        'severity': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Duration'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint64'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.UnboundedString()),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.name = kwargs.get('name', str())
        from builtin_interfaces.msg import Duration
        self.duration = kwargs.get('duration', Duration())
        self.code = kwargs.get('code', int())
        self.uid = kwargs.get('uid', int())
        self.error_message = kwargs.get('error_message', str())
        self.attributes = kwargs.get('attributes', [])
        self.severity = kwargs.get('severity', int())

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
        if self.name != other.name:
            return False
        if self.duration != other.duration:
            return False
        if self.code != other.code:
            return False
        if self.uid != other.uid:
            return False
        if self.error_message != other.error_message:
            return False
        if self.attributes != other.attributes:
            return False
        if self.severity != other.severity:
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
    def name(self):
        """Message field 'name'."""
        return self._name

    @name.setter
    def name(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'name' field must be of type 'str'"
        self._name = value

    @builtins.property
    def duration(self):
        """Message field 'duration'."""
        return self._duration

    @duration.setter
    def duration(self, value):
        if __debug__:
            from builtin_interfaces.msg import Duration
            assert \
                isinstance(value, Duration), \
                "The 'duration' field must be a sub message of type 'Duration'"
        self._duration = value

    @builtins.property
    def code(self):
        """Message field 'code'."""
        return self._code

    @code.setter
    def code(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'code' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'code' field must be an integer in [-2147483648, 2147483647]"
        self._code = value

    @builtins.property
    def uid(self):
        """Message field 'uid'."""
        return self._uid

    @uid.setter
    def uid(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'uid' field must be of type 'int'"
            assert value >= 0 and value < 18446744073709551616, \
                "The 'uid' field must be an unsigned integer in [0, 18446744073709551615]"
        self._uid = value

    @builtins.property
    def error_message(self):
        """Message field 'error_message'."""
        return self._error_message

    @error_message.setter
    def error_message(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'error_message' field must be of type 'str'"
        self._error_message = value

    @builtins.property
    def attributes(self):
        """Message field 'attributes'."""
        return self._attributes

    @attributes.setter
    def attributes(self, value):
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
                 all(isinstance(v, str) for v in value) and
                 True), \
                "The 'attributes' field must be a set or sequence and each value of type 'str'"
        self._attributes = value

    @builtins.property
    def severity(self):
        """Message field 'severity'."""
        return self._severity

    @severity.setter
    def severity(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'severity' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'severity' field must be an unsigned integer in [0, 255]"
        self._severity = value
