# generated from rosidl_generator_py/resource/_idl.py.em
# with input from spot_msgs:msg/FootState.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_FootState(type):
    """Metaclass of message 'FootState'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'CONTACT_UNKNOWN': 0,
        'CONTACT_MADE': 1,
        'CONTACT_LOST': 2,
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
                'spot_msgs.msg.FootState')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__foot_state
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__foot_state
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__foot_state
            cls._TYPE_SUPPORT = module.type_support_msg__msg__foot_state
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__foot_state

            from geometry_msgs.msg import Point
            if Point.__class__._TYPE_SUPPORT is None:
                Point.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'CONTACT_UNKNOWN': cls.__constants['CONTACT_UNKNOWN'],
            'CONTACT_MADE': cls.__constants['CONTACT_MADE'],
            'CONTACT_LOST': cls.__constants['CONTACT_LOST'],
        }

    @property
    def CONTACT_UNKNOWN(self):
        """Message constant 'CONTACT_UNKNOWN'."""
        return Metaclass_FootState.__constants['CONTACT_UNKNOWN']

    @property
    def CONTACT_MADE(self):
        """Message constant 'CONTACT_MADE'."""
        return Metaclass_FootState.__constants['CONTACT_MADE']

    @property
    def CONTACT_LOST(self):
        """Message constant 'CONTACT_LOST'."""
        return Metaclass_FootState.__constants['CONTACT_LOST']


class FootState(metaclass=Metaclass_FootState):
    """
    Message class 'FootState'.

    Constants:
      CONTACT_UNKNOWN
      CONTACT_MADE
      CONTACT_LOST
    """

    __slots__ = [
        '_foot_position_rt_body',
        '_contact',
    ]

    _fields_and_field_types = {
        'foot_position_rt_body': 'geometry_msgs/Point',
        'contact': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Point'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from geometry_msgs.msg import Point
        self.foot_position_rt_body = kwargs.get('foot_position_rt_body', Point())
        self.contact = kwargs.get('contact', int())

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
        if self.foot_position_rt_body != other.foot_position_rt_body:
            return False
        if self.contact != other.contact:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def foot_position_rt_body(self):
        """Message field 'foot_position_rt_body'."""
        return self._foot_position_rt_body

    @foot_position_rt_body.setter
    def foot_position_rt_body(self, value):
        if __debug__:
            from geometry_msgs.msg import Point
            assert \
                isinstance(value, Point), \
                "The 'foot_position_rt_body' field must be a sub message of type 'Point'"
        self._foot_position_rt_body = value

    @builtins.property
    def contact(self):
        """Message field 'contact'."""
        return self._contact

    @contact.setter
    def contact(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'contact' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'contact' field must be an unsigned integer in [0, 255]"
        self._contact = value
