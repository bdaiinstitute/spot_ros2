# generated from rosidl_generator_py/resource/_idl.py.em
# with input from spot_msgs:msg/LeaseOwner.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_LeaseOwner(type):
    """Metaclass of message 'LeaseOwner'."""

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
                'spot_msgs.msg.LeaseOwner')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__lease_owner
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__lease_owner
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__lease_owner
            cls._TYPE_SUPPORT = module.type_support_msg__msg__lease_owner
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__lease_owner

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class LeaseOwner(metaclass=Metaclass_LeaseOwner):
    """Message class 'LeaseOwner'."""

    __slots__ = [
        '_client_name',
        '_user_name',
    ]

    _fields_and_field_types = {
        'client_name': 'string',
        'user_name': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.client_name = kwargs.get('client_name', str())
        self.user_name = kwargs.get('user_name', str())

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
        if self.client_name != other.client_name:
            return False
        if self.user_name != other.user_name:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def client_name(self):
        """Message field 'client_name'."""
        return self._client_name

    @client_name.setter
    def client_name(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'client_name' field must be of type 'str'"
        self._client_name = value

    @builtins.property
    def user_name(self):
        """Message field 'user_name'."""
        return self._user_name

    @user_name.setter
    def user_name(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'user_name' field must be of type 'str'"
        self._user_name = value
