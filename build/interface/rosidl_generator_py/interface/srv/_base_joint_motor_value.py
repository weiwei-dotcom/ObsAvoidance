# generated from rosidl_generator_py/resource/_idl.py.em
# with input from interface:srv/BaseJointMotorValue.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_BaseJointMotorValue_Request(type):
    """Metaclass of message 'BaseJointMotorValue_Request'."""

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
            module = import_type_support('interface')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'interface.srv.BaseJointMotorValue_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__base_joint_motor_value__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__base_joint_motor_value__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__base_joint_motor_value__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__base_joint_motor_value__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__base_joint_motor_value__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class BaseJointMotorValue_Request(metaclass=Metaclass_BaseJointMotorValue_Request):
    """Message class 'BaseJointMotorValue_Request'."""

    __slots__ = [
    ]

    _fields_and_field_types = {
    }

    SLOT_TYPES = (
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))

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
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)


# Import statements for member types

# already imported above
# import rosidl_parser.definition


class Metaclass_BaseJointMotorValue_Response(type):
    """Metaclass of message 'BaseJointMotorValue_Response'."""

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
            module = import_type_support('interface')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'interface.srv.BaseJointMotorValue_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__base_joint_motor_value__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__base_joint_motor_value__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__base_joint_motor_value__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__base_joint_motor_value__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__base_joint_motor_value__response

            from geometry_msgs.msg import Point
            if Point.__class__._TYPE_SUPPORT is None:
                Point.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class BaseJointMotorValue_Response(metaclass=Metaclass_BaseJointMotorValue_Response):
    """Message class 'BaseJointMotorValue_Response'."""

    __slots__ = [
        '_cable_expend_value',
        '_base_advance_value',
        '_flag_base_out_range',
        '_flag_arrived_target',
    ]

    _fields_and_field_types = {
        'cable_expend_value': 'sequence<geometry_msgs/Point>',
        'base_advance_value': 'double',
        'flag_base_out_range': 'boolean',
        'flag_arrived_target': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Point')),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.cable_expend_value = kwargs.get('cable_expend_value', [])
        self.base_advance_value = kwargs.get('base_advance_value', float())
        self.flag_base_out_range = kwargs.get('flag_base_out_range', bool())
        self.flag_arrived_target = kwargs.get('flag_arrived_target', bool())

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
        if self.cable_expend_value != other.cable_expend_value:
            return False
        if self.base_advance_value != other.base_advance_value:
            return False
        if self.flag_base_out_range != other.flag_base_out_range:
            return False
        if self.flag_arrived_target != other.flag_arrived_target:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def cable_expend_value(self):
        """Message field 'cable_expend_value'."""
        return self._cable_expend_value

    @cable_expend_value.setter
    def cable_expend_value(self, value):
        if __debug__:
            from geometry_msgs.msg import Point
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
                 all(isinstance(v, Point) for v in value) and
                 True), \
                "The 'cable_expend_value' field must be a set or sequence and each value of type 'Point'"
        self._cable_expend_value = value

    @property
    def base_advance_value(self):
        """Message field 'base_advance_value'."""
        return self._base_advance_value

    @base_advance_value.setter
    def base_advance_value(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'base_advance_value' field must be of type 'float'"
        self._base_advance_value = value

    @property
    def flag_base_out_range(self):
        """Message field 'flag_base_out_range'."""
        return self._flag_base_out_range

    @flag_base_out_range.setter
    def flag_base_out_range(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'flag_base_out_range' field must be of type 'bool'"
        self._flag_base_out_range = value

    @property
    def flag_arrived_target(self):
        """Message field 'flag_arrived_target'."""
        return self._flag_arrived_target

    @flag_arrived_target.setter
    def flag_arrived_target(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'flag_arrived_target' field must be of type 'bool'"
        self._flag_arrived_target = value


class Metaclass_BaseJointMotorValue(type):
    """Metaclass of service 'BaseJointMotorValue'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('interface')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'interface.srv.BaseJointMotorValue')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__base_joint_motor_value

            from interface.srv import _base_joint_motor_value
            if _base_joint_motor_value.Metaclass_BaseJointMotorValue_Request._TYPE_SUPPORT is None:
                _base_joint_motor_value.Metaclass_BaseJointMotorValue_Request.__import_type_support__()
            if _base_joint_motor_value.Metaclass_BaseJointMotorValue_Response._TYPE_SUPPORT is None:
                _base_joint_motor_value.Metaclass_BaseJointMotorValue_Response.__import_type_support__()


class BaseJointMotorValue(metaclass=Metaclass_BaseJointMotorValue):
    from interface.srv._base_joint_motor_value import BaseJointMotorValue_Request as Request
    from interface.srv._base_joint_motor_value import BaseJointMotorValue_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
