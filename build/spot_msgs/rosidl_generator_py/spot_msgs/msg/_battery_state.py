# generated from rosidl_generator_py/resource/_idl.py.em
# with input from spot_msgs:msg/BatteryState.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'temperatures'
import array  # noqa: E402, I100

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_BatteryState(type):
    """Metaclass of message 'BatteryState'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'STATUS_UNKNOWN': 0,
        'STATUS_MISSING': 1,
        'STATUS_CHARGING': 2,
        'STATUS_DISCHARGING': 3,
        'STATUS_BOOTING': 4,
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
                'spot_msgs.msg.BatteryState')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__battery_state
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__battery_state
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__battery_state
            cls._TYPE_SUPPORT = module.type_support_msg__msg__battery_state
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__battery_state

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
            'STATUS_UNKNOWN': cls.__constants['STATUS_UNKNOWN'],
            'STATUS_MISSING': cls.__constants['STATUS_MISSING'],
            'STATUS_CHARGING': cls.__constants['STATUS_CHARGING'],
            'STATUS_DISCHARGING': cls.__constants['STATUS_DISCHARGING'],
            'STATUS_BOOTING': cls.__constants['STATUS_BOOTING'],
        }

    @property
    def STATUS_UNKNOWN(self):
        """Message constant 'STATUS_UNKNOWN'."""
        return Metaclass_BatteryState.__constants['STATUS_UNKNOWN']

    @property
    def STATUS_MISSING(self):
        """Message constant 'STATUS_MISSING'."""
        return Metaclass_BatteryState.__constants['STATUS_MISSING']

    @property
    def STATUS_CHARGING(self):
        """Message constant 'STATUS_CHARGING'."""
        return Metaclass_BatteryState.__constants['STATUS_CHARGING']

    @property
    def STATUS_DISCHARGING(self):
        """Message constant 'STATUS_DISCHARGING'."""
        return Metaclass_BatteryState.__constants['STATUS_DISCHARGING']

    @property
    def STATUS_BOOTING(self):
        """Message constant 'STATUS_BOOTING'."""
        return Metaclass_BatteryState.__constants['STATUS_BOOTING']


class BatteryState(metaclass=Metaclass_BatteryState):
    """
    Message class 'BatteryState'.

    Constants:
      STATUS_UNKNOWN
      STATUS_MISSING
      STATUS_CHARGING
      STATUS_DISCHARGING
      STATUS_BOOTING
    """

    __slots__ = [
        '_header',
        '_identifier',
        '_charge_percentage',
        '_estimated_runtime',
        '_current',
        '_voltage',
        '_temperatures',
        '_status',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'identifier': 'string',
        'charge_percentage': 'double',
        'estimated_runtime': 'builtin_interfaces/Duration',
        'current': 'double',
        'voltage': 'double',
        'temperatures': 'sequence<double>',
        'status': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Duration'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('double')),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.identifier = kwargs.get('identifier', str())
        self.charge_percentage = kwargs.get('charge_percentage', float())
        from builtin_interfaces.msg import Duration
        self.estimated_runtime = kwargs.get('estimated_runtime', Duration())
        self.current = kwargs.get('current', float())
        self.voltage = kwargs.get('voltage', float())
        self.temperatures = array.array('d', kwargs.get('temperatures', []))
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
        if self.identifier != other.identifier:
            return False
        if self.charge_percentage != other.charge_percentage:
            return False
        if self.estimated_runtime != other.estimated_runtime:
            return False
        if self.current != other.current:
            return False
        if self.voltage != other.voltage:
            return False
        if self.temperatures != other.temperatures:
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
    def identifier(self):
        """Message field 'identifier'."""
        return self._identifier

    @identifier.setter
    def identifier(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'identifier' field must be of type 'str'"
        self._identifier = value

    @builtins.property
    def charge_percentage(self):
        """Message field 'charge_percentage'."""
        return self._charge_percentage

    @charge_percentage.setter
    def charge_percentage(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'charge_percentage' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'charge_percentage' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._charge_percentage = value

    @builtins.property
    def estimated_runtime(self):
        """Message field 'estimated_runtime'."""
        return self._estimated_runtime

    @estimated_runtime.setter
    def estimated_runtime(self, value):
        if __debug__:
            from builtin_interfaces.msg import Duration
            assert \
                isinstance(value, Duration), \
                "The 'estimated_runtime' field must be a sub message of type 'Duration'"
        self._estimated_runtime = value

    @builtins.property
    def current(self):
        """Message field 'current'."""
        return self._current

    @current.setter
    def current(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'current' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'current' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._current = value

    @builtins.property
    def voltage(self):
        """Message field 'voltage'."""
        return self._voltage

    @voltage.setter
    def voltage(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'voltage' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'voltage' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._voltage = value

    @builtins.property
    def temperatures(self):
        """Message field 'temperatures'."""
        return self._temperatures

    @temperatures.setter
    def temperatures(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'd', \
                "The 'temperatures' array.array() must have the type code of 'd'"
            self._temperatures = value
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
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'temperatures' field must be a set or sequence and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._temperatures = array.array('d', value)

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
