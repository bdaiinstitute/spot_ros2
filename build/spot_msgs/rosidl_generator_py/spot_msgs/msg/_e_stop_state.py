# generated from rosidl_generator_py/resource/_idl.py.em
# with input from spot_msgs:msg/EStopState.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_EStopState(type):
    """Metaclass of message 'EStopState'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'TYPE_UNKNOWN': 0,
        'TYPE_HARDWARE': 1,
        'TYPE_SOFTWARE': 2,
        'STATE_UNKNOWN': 0,
        'STATE_ESTOPPED': 1,
        'STATE_NOT_ESTOPPED': 2,
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
                'spot_msgs.msg.EStopState')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__e_stop_state
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__e_stop_state
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__e_stop_state
            cls._TYPE_SUPPORT = module.type_support_msg__msg__e_stop_state
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__e_stop_state

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'TYPE_UNKNOWN': cls.__constants['TYPE_UNKNOWN'],
            'TYPE_HARDWARE': cls.__constants['TYPE_HARDWARE'],
            'TYPE_SOFTWARE': cls.__constants['TYPE_SOFTWARE'],
            'STATE_UNKNOWN': cls.__constants['STATE_UNKNOWN'],
            'STATE_ESTOPPED': cls.__constants['STATE_ESTOPPED'],
            'STATE_NOT_ESTOPPED': cls.__constants['STATE_NOT_ESTOPPED'],
        }

    @property
    def TYPE_UNKNOWN(self):
        """Message constant 'TYPE_UNKNOWN'."""
        return Metaclass_EStopState.__constants['TYPE_UNKNOWN']

    @property
    def TYPE_HARDWARE(self):
        """Message constant 'TYPE_HARDWARE'."""
        return Metaclass_EStopState.__constants['TYPE_HARDWARE']

    @property
    def TYPE_SOFTWARE(self):
        """Message constant 'TYPE_SOFTWARE'."""
        return Metaclass_EStopState.__constants['TYPE_SOFTWARE']

    @property
    def STATE_UNKNOWN(self):
        """Message constant 'STATE_UNKNOWN'."""
        return Metaclass_EStopState.__constants['STATE_UNKNOWN']

    @property
    def STATE_ESTOPPED(self):
        """Message constant 'STATE_ESTOPPED'."""
        return Metaclass_EStopState.__constants['STATE_ESTOPPED']

    @property
    def STATE_NOT_ESTOPPED(self):
        """Message constant 'STATE_NOT_ESTOPPED'."""
        return Metaclass_EStopState.__constants['STATE_NOT_ESTOPPED']


class EStopState(metaclass=Metaclass_EStopState):
    """
    Message class 'EStopState'.

    Constants:
      TYPE_UNKNOWN
      TYPE_HARDWARE
      TYPE_SOFTWARE
      STATE_UNKNOWN
      STATE_ESTOPPED
      STATE_NOT_ESTOPPED
    """

    __slots__ = [
        '_header',
        '_name',
        '_type',
        '_state',
        '_state_description',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'name': 'string',
        'type': 'uint8',
        'state': 'uint8',
        'state_description': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.name = kwargs.get('name', str())
        self.type = kwargs.get('type', int())
        self.state = kwargs.get('state', int())
        self.state_description = kwargs.get('state_description', str())

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
        if self.type != other.type:
            return False
        if self.state != other.state:
            return False
        if self.state_description != other.state_description:
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

    @builtins.property  # noqa: A003
    def type(self):  # noqa: A003
        """Message field 'type'."""
        return self._type

    @type.setter  # noqa: A003
    def type(self, value):  # noqa: A003
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'type' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'type' field must be an unsigned integer in [0, 255]"
        self._type = value

    @builtins.property
    def state(self):
        """Message field 'state'."""
        return self._state

    @state.setter
    def state(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'state' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'state' field must be an unsigned integer in [0, 255]"
        self._state = value

    @builtins.property
    def state_description(self):
        """Message field 'state_description'."""
        return self._state_description

    @state_description.setter
    def state_description(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'state_description' field must be of type 'str'"
        self._state_description = value
