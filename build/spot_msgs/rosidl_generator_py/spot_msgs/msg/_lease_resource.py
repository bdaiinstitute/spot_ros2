# generated from rosidl_generator_py/resource/_idl.py.em
# with input from spot_msgs:msg/LeaseResource.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_LeaseResource(type):
    """Metaclass of message 'LeaseResource'."""

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
                'spot_msgs.msg.LeaseResource')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__lease_resource
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__lease_resource
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__lease_resource
            cls._TYPE_SUPPORT = module.type_support_msg__msg__lease_resource
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__lease_resource

            from spot_msgs.msg import Lease
            if Lease.__class__._TYPE_SUPPORT is None:
                Lease.__class__.__import_type_support__()

            from spot_msgs.msg import LeaseOwner
            if LeaseOwner.__class__._TYPE_SUPPORT is None:
                LeaseOwner.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class LeaseResource(metaclass=Metaclass_LeaseResource):
    """Message class 'LeaseResource'."""

    __slots__ = [
        '_resource',
        '_lease',
        '_lease_owner',
    ]

    _fields_and_field_types = {
        'resource': 'string',
        'lease': 'spot_msgs/Lease',
        'lease_owner': 'spot_msgs/LeaseOwner',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['spot_msgs', 'msg'], 'Lease'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['spot_msgs', 'msg'], 'LeaseOwner'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.resource = kwargs.get('resource', str())
        from spot_msgs.msg import Lease
        self.lease = kwargs.get('lease', Lease())
        from spot_msgs.msg import LeaseOwner
        self.lease_owner = kwargs.get('lease_owner', LeaseOwner())

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
        if self.resource != other.resource:
            return False
        if self.lease != other.lease:
            return False
        if self.lease_owner != other.lease_owner:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def resource(self):
        """Message field 'resource'."""
        return self._resource

    @resource.setter
    def resource(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'resource' field must be of type 'str'"
        self._resource = value

    @builtins.property
    def lease(self):
        """Message field 'lease'."""
        return self._lease

    @lease.setter
    def lease(self, value):
        if __debug__:
            from spot_msgs.msg import Lease
            assert \
                isinstance(value, Lease), \
                "The 'lease' field must be a sub message of type 'Lease'"
        self._lease = value

    @builtins.property
    def lease_owner(self):
        """Message field 'lease_owner'."""
        return self._lease_owner

    @lease_owner.setter
    def lease_owner(self, value):
        if __debug__:
            from spot_msgs.msg import LeaseOwner
            assert \
                isinstance(value, LeaseOwner), \
                "The 'lease_owner' field must be a sub message of type 'LeaseOwner'"
        self._lease_owner = value
