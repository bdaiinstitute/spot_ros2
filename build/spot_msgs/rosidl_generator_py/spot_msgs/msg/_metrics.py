# generated from rosidl_generator_py/resource/_idl.py.em
# with input from spot_msgs:msg/Metrics.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Metrics(type):
    """Metaclass of message 'Metrics'."""

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
                'spot_msgs.msg.Metrics')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__metrics
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__metrics
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__metrics
            cls._TYPE_SUPPORT = module.type_support_msg__msg__metrics
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__metrics

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
        }


class Metrics(metaclass=Metaclass_Metrics):
    """Message class 'Metrics'."""

    __slots__ = [
        '_header',
        '_distance',
        '_gait_cycles',
        '_time_moving',
        '_electric_power',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'distance': 'float',
        'gait_cycles': 'int32',
        'time_moving': 'builtin_interfaces/Duration',
        'electric_power': 'builtin_interfaces/Duration',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Duration'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Duration'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.distance = kwargs.get('distance', float())
        self.gait_cycles = kwargs.get('gait_cycles', int())
        from builtin_interfaces.msg import Duration
        self.time_moving = kwargs.get('time_moving', Duration())
        from builtin_interfaces.msg import Duration
        self.electric_power = kwargs.get('electric_power', Duration())

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
        if self.distance != other.distance:
            return False
        if self.gait_cycles != other.gait_cycles:
            return False
        if self.time_moving != other.time_moving:
            return False
        if self.electric_power != other.electric_power:
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
    def distance(self):
        """Message field 'distance'."""
        return self._distance

    @distance.setter
    def distance(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'distance' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'distance' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._distance = value

    @builtins.property
    def gait_cycles(self):
        """Message field 'gait_cycles'."""
        return self._gait_cycles

    @gait_cycles.setter
    def gait_cycles(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'gait_cycles' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'gait_cycles' field must be an integer in [-2147483648, 2147483647]"
        self._gait_cycles = value

    @builtins.property
    def time_moving(self):
        """Message field 'time_moving'."""
        return self._time_moving

    @time_moving.setter
    def time_moving(self, value):
        if __debug__:
            from builtin_interfaces.msg import Duration
            assert \
                isinstance(value, Duration), \
                "The 'time_moving' field must be a sub message of type 'Duration'"
        self._time_moving = value

    @builtins.property
    def electric_power(self):
        """Message field 'electric_power'."""
        return self._electric_power

    @electric_power.setter
    def electric_power(self, value):
        if __debug__:
            from builtin_interfaces.msg import Duration
            assert \
                isinstance(value, Duration), \
                "The 'electric_power' field must be a sub message of type 'Duration'"
        self._electric_power = value
