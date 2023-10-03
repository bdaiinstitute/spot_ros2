# generated from rosidl_generator_py/resource/_idl.py.em
# with input from spot_msgs:msg/PowerState.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_PowerState(type):
    """Metaclass of message 'PowerState'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'STATE_UNKNOWN': 0,
        'STATE_OFF': 1,
        'STATE_ON': 2,
        'STATE_POWERING_ON': 3,
        'STATE_POWERING_OFF': 4,
        'STATE_ERROR': 5,
        'STATE_UNKNOWN_SHORE_POWER': 0,
        'STATE_ON_SHORE_POWER': 1,
        'STATE_OFF_SHORE_POWER': 2,
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
                'spot_msgs.msg.PowerState')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__power_state
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__power_state
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__power_state
            cls._TYPE_SUPPORT = module.type_support_msg__msg__power_state
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__power_state

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
            'STATE_UNKNOWN': cls.__constants['STATE_UNKNOWN'],
            'STATE_OFF': cls.__constants['STATE_OFF'],
            'STATE_ON': cls.__constants['STATE_ON'],
            'STATE_POWERING_ON': cls.__constants['STATE_POWERING_ON'],
            'STATE_POWERING_OFF': cls.__constants['STATE_POWERING_OFF'],
            'STATE_ERROR': cls.__constants['STATE_ERROR'],
            'STATE_UNKNOWN_SHORE_POWER': cls.__constants['STATE_UNKNOWN_SHORE_POWER'],
            'STATE_ON_SHORE_POWER': cls.__constants['STATE_ON_SHORE_POWER'],
            'STATE_OFF_SHORE_POWER': cls.__constants['STATE_OFF_SHORE_POWER'],
        }

    @property
    def STATE_UNKNOWN(self):
        """Message constant 'STATE_UNKNOWN'."""
        return Metaclass_PowerState.__constants['STATE_UNKNOWN']

    @property
    def STATE_OFF(self):
        """Message constant 'STATE_OFF'."""
        return Metaclass_PowerState.__constants['STATE_OFF']

    @property
    def STATE_ON(self):
        """Message constant 'STATE_ON'."""
        return Metaclass_PowerState.__constants['STATE_ON']

    @property
    def STATE_POWERING_ON(self):
        """Message constant 'STATE_POWERING_ON'."""
        return Metaclass_PowerState.__constants['STATE_POWERING_ON']

    @property
    def STATE_POWERING_OFF(self):
        """Message constant 'STATE_POWERING_OFF'."""
        return Metaclass_PowerState.__constants['STATE_POWERING_OFF']

    @property
    def STATE_ERROR(self):
        """Message constant 'STATE_ERROR'."""
        return Metaclass_PowerState.__constants['STATE_ERROR']

    @property
    def STATE_UNKNOWN_SHORE_POWER(self):
        """Message constant 'STATE_UNKNOWN_SHORE_POWER'."""
        return Metaclass_PowerState.__constants['STATE_UNKNOWN_SHORE_POWER']

    @property
    def STATE_ON_SHORE_POWER(self):
        """Message constant 'STATE_ON_SHORE_POWER'."""
        return Metaclass_PowerState.__constants['STATE_ON_SHORE_POWER']

    @property
    def STATE_OFF_SHORE_POWER(self):
        """Message constant 'STATE_OFF_SHORE_POWER'."""
        return Metaclass_PowerState.__constants['STATE_OFF_SHORE_POWER']


class PowerState(metaclass=Metaclass_PowerState):
    """
    Message class 'PowerState'.

    Constants:
      STATE_UNKNOWN
      STATE_OFF
      STATE_ON
      STATE_POWERING_ON
      STATE_POWERING_OFF
      STATE_ERROR
      STATE_UNKNOWN_SHORE_POWER
      STATE_ON_SHORE_POWER
      STATE_OFF_SHORE_POWER
    """

    __slots__ = [
        '_header',
        '_motor_power_state',
        '_shore_power_state',
        '_locomotion_charge_percentage',
        '_locomotion_estimated_runtime',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'motor_power_state': 'uint8',
        'shore_power_state': 'uint8',
        'locomotion_charge_percentage': 'double',
        'locomotion_estimated_runtime': 'builtin_interfaces/Duration',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Duration'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.motor_power_state = kwargs.get('motor_power_state', int())
        self.shore_power_state = kwargs.get('shore_power_state', int())
        self.locomotion_charge_percentage = kwargs.get('locomotion_charge_percentage', float())
        from builtin_interfaces.msg import Duration
        self.locomotion_estimated_runtime = kwargs.get('locomotion_estimated_runtime', Duration())

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
        if self.motor_power_state != other.motor_power_state:
            return False
        if self.shore_power_state != other.shore_power_state:
            return False
        if self.locomotion_charge_percentage != other.locomotion_charge_percentage:
            return False
        if self.locomotion_estimated_runtime != other.locomotion_estimated_runtime:
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
    def motor_power_state(self):
        """Message field 'motor_power_state'."""
        return self._motor_power_state

    @motor_power_state.setter
    def motor_power_state(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'motor_power_state' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'motor_power_state' field must be an unsigned integer in [0, 255]"
        self._motor_power_state = value

    @builtins.property
    def shore_power_state(self):
        """Message field 'shore_power_state'."""
        return self._shore_power_state

    @shore_power_state.setter
    def shore_power_state(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'shore_power_state' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'shore_power_state' field must be an unsigned integer in [0, 255]"
        self._shore_power_state = value

    @builtins.property
    def locomotion_charge_percentage(self):
        """Message field 'locomotion_charge_percentage'."""
        return self._locomotion_charge_percentage

    @locomotion_charge_percentage.setter
    def locomotion_charge_percentage(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'locomotion_charge_percentage' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'locomotion_charge_percentage' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._locomotion_charge_percentage = value

    @builtins.property
    def locomotion_estimated_runtime(self):
        """Message field 'locomotion_estimated_runtime'."""
        return self._locomotion_estimated_runtime

    @locomotion_estimated_runtime.setter
    def locomotion_estimated_runtime(self, value):
        if __debug__:
            from builtin_interfaces.msg import Duration
            assert \
                isinstance(value, Duration), \
                "The 'locomotion_estimated_runtime' field must be a sub message of type 'Duration'"
        self._locomotion_estimated_runtime = value
