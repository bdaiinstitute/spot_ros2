# generated from rosidl_generator_py/resource/_idl.py.em
# with input from spot_msgs:msg/Feedback.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Feedback(type):
    """Metaclass of message 'Feedback'."""

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
                'spot_msgs.msg.Feedback')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__feedback
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__feedback
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__feedback
            cls._TYPE_SUPPORT = module.type_support_msg__msg__feedback
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__feedback

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Feedback(metaclass=Metaclass_Feedback):
    """Message class 'Feedback'."""

    __slots__ = [
        '_standing',
        '_sitting',
        '_moving',
        '_serial_number',
        '_species',
        '_version',
        '_nickname',
        '_computer_serial_number',
    ]

    _fields_and_field_types = {
        'standing': 'boolean',
        'sitting': 'boolean',
        'moving': 'boolean',
        'serial_number': 'string',
        'species': 'string',
        'version': 'string',
        'nickname': 'string',
        'computer_serial_number': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.standing = kwargs.get('standing', bool())
        self.sitting = kwargs.get('sitting', bool())
        self.moving = kwargs.get('moving', bool())
        self.serial_number = kwargs.get('serial_number', str())
        self.species = kwargs.get('species', str())
        self.version = kwargs.get('version', str())
        self.nickname = kwargs.get('nickname', str())
        self.computer_serial_number = kwargs.get('computer_serial_number', str())

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
        if self.standing != other.standing:
            return False
        if self.sitting != other.sitting:
            return False
        if self.moving != other.moving:
            return False
        if self.serial_number != other.serial_number:
            return False
        if self.species != other.species:
            return False
        if self.version != other.version:
            return False
        if self.nickname != other.nickname:
            return False
        if self.computer_serial_number != other.computer_serial_number:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def standing(self):
        """Message field 'standing'."""
        return self._standing

    @standing.setter
    def standing(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'standing' field must be of type 'bool'"
        self._standing = value

    @builtins.property
    def sitting(self):
        """Message field 'sitting'."""
        return self._sitting

    @sitting.setter
    def sitting(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'sitting' field must be of type 'bool'"
        self._sitting = value

    @builtins.property
    def moving(self):
        """Message field 'moving'."""
        return self._moving

    @moving.setter
    def moving(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'moving' field must be of type 'bool'"
        self._moving = value

    @builtins.property
    def serial_number(self):
        """Message field 'serial_number'."""
        return self._serial_number

    @serial_number.setter
    def serial_number(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'serial_number' field must be of type 'str'"
        self._serial_number = value

    @builtins.property
    def species(self):
        """Message field 'species'."""
        return self._species

    @species.setter
    def species(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'species' field must be of type 'str'"
        self._species = value

    @builtins.property
    def version(self):
        """Message field 'version'."""
        return self._version

    @version.setter
    def version(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'version' field must be of type 'str'"
        self._version = value

    @builtins.property
    def nickname(self):
        """Message field 'nickname'."""
        return self._nickname

    @nickname.setter
    def nickname(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'nickname' field must be of type 'str'"
        self._nickname = value

    @builtins.property
    def computer_serial_number(self):
        """Message field 'computer_serial_number'."""
        return self._computer_serial_number

    @computer_serial_number.setter
    def computer_serial_number(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'computer_serial_number' field must be of type 'str'"
        self._computer_serial_number = value
