# generated from rosidl_generator_py/resource/_idl.py.em
# with input from spot_msgs:msg/WiFiState.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_WiFiState(type):
    """Metaclass of message 'WiFiState'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'MODE_UNKNOWN': 0,
        'MODE_ACCESS_POINT': 1,
        'MODE_CLIENT': 2,
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
                'spot_msgs.msg.WiFiState')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__wi_fi_state
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__wi_fi_state
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__wi_fi_state
            cls._TYPE_SUPPORT = module.type_support_msg__msg__wi_fi_state
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__wi_fi_state

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'MODE_UNKNOWN': cls.__constants['MODE_UNKNOWN'],
            'MODE_ACCESS_POINT': cls.__constants['MODE_ACCESS_POINT'],
            'MODE_CLIENT': cls.__constants['MODE_CLIENT'],
        }

    @property
    def MODE_UNKNOWN(self):
        """Message constant 'MODE_UNKNOWN'."""
        return Metaclass_WiFiState.__constants['MODE_UNKNOWN']

    @property
    def MODE_ACCESS_POINT(self):
        """Message constant 'MODE_ACCESS_POINT'."""
        return Metaclass_WiFiState.__constants['MODE_ACCESS_POINT']

    @property
    def MODE_CLIENT(self):
        """Message constant 'MODE_CLIENT'."""
        return Metaclass_WiFiState.__constants['MODE_CLIENT']


class WiFiState(metaclass=Metaclass_WiFiState):
    """
    Message class 'WiFiState'.

    Constants:
      MODE_UNKNOWN
      MODE_ACCESS_POINT
      MODE_CLIENT
    """

    __slots__ = [
        '_current_mode',
        '_essid',
    ]

    _fields_and_field_types = {
        'current_mode': 'uint8',
        'essid': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.current_mode = kwargs.get('current_mode', int())
        self.essid = kwargs.get('essid', str())

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
        if self.current_mode != other.current_mode:
            return False
        if self.essid != other.essid:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def current_mode(self):
        """Message field 'current_mode'."""
        return self._current_mode

    @current_mode.setter
    def current_mode(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'current_mode' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'current_mode' field must be an unsigned integer in [0, 255]"
        self._current_mode = value

    @builtins.property
    def essid(self):
        """Message field 'essid'."""
        return self._essid

    @essid.setter
    def essid(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'essid' field must be of type 'str'"
        self._essid = value
