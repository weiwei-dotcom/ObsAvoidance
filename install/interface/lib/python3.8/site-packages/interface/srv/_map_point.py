# generated from rosidl_generator_py/resource/_idl.py.em
# with input from interface:srv/MapPoint.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_MapPoint_Request(type):
    """Metaclass of message 'MapPoint_Request'."""

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
                'interface.srv.MapPoint_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__map_point__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__map_point__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__map_point__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__map_point__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__map_point__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class MapPoint_Request(metaclass=Metaclass_MapPoint_Request):
    """Message class 'MapPoint_Request'."""

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


class Metaclass_MapPoint_Response(type):
    """Metaclass of message 'MapPoint_Response'."""

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
                'interface.srv.MapPoint_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__map_point__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__map_point__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__map_point__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__map_point__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__map_point__response

            from geometry_msgs.msg import Pose
            if Pose.__class__._TYPE_SUPPORT is None:
                Pose.__class__.__import_type_support__()

            from sensor_msgs.msg import Image
            if Image.__class__._TYPE_SUPPORT is None:
                Image.__class__.__import_type_support__()

            from sensor_msgs.msg import PointCloud2
            if PointCloud2.__class__._TYPE_SUPPORT is None:
                PointCloud2.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class MapPoint_Response(metaclass=Metaclass_MapPoint_Response):
    """Message class 'MapPoint_Response'."""

    __slots__ = [
        '_point_cloud',
        '_img',
        '_cam_pose',
        '_world2cam',
    ]

    _fields_and_field_types = {
        'point_cloud': 'sensor_msgs/PointCloud2',
        'img': 'sensor_msgs/Image',
        'cam_pose': 'geometry_msgs/Pose',
        'world2cam': 'geometry_msgs/Pose',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['sensor_msgs', 'msg'], 'PointCloud2'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['sensor_msgs', 'msg'], 'Image'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Pose'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Pose'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from sensor_msgs.msg import PointCloud2
        self.point_cloud = kwargs.get('point_cloud', PointCloud2())
        from sensor_msgs.msg import Image
        self.img = kwargs.get('img', Image())
        from geometry_msgs.msg import Pose
        self.cam_pose = kwargs.get('cam_pose', Pose())
        from geometry_msgs.msg import Pose
        self.world2cam = kwargs.get('world2cam', Pose())

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
        if self.point_cloud != other.point_cloud:
            return False
        if self.img != other.img:
            return False
        if self.cam_pose != other.cam_pose:
            return False
        if self.world2cam != other.world2cam:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def point_cloud(self):
        """Message field 'point_cloud'."""
        return self._point_cloud

    @point_cloud.setter
    def point_cloud(self, value):
        if __debug__:
            from sensor_msgs.msg import PointCloud2
            assert \
                isinstance(value, PointCloud2), \
                "The 'point_cloud' field must be a sub message of type 'PointCloud2'"
        self._point_cloud = value

    @property
    def img(self):
        """Message field 'img'."""
        return self._img

    @img.setter
    def img(self, value):
        if __debug__:
            from sensor_msgs.msg import Image
            assert \
                isinstance(value, Image), \
                "The 'img' field must be a sub message of type 'Image'"
        self._img = value

    @property
    def cam_pose(self):
        """Message field 'cam_pose'."""
        return self._cam_pose

    @cam_pose.setter
    def cam_pose(self, value):
        if __debug__:
            from geometry_msgs.msg import Pose
            assert \
                isinstance(value, Pose), \
                "The 'cam_pose' field must be a sub message of type 'Pose'"
        self._cam_pose = value

    @property
    def world2cam(self):
        """Message field 'world2cam'."""
        return self._world2cam

    @world2cam.setter
    def world2cam(self, value):
        if __debug__:
            from geometry_msgs.msg import Pose
            assert \
                isinstance(value, Pose), \
                "The 'world2cam' field must be a sub message of type 'Pose'"
        self._world2cam = value


class Metaclass_MapPoint(type):
    """Metaclass of service 'MapPoint'."""

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
                'interface.srv.MapPoint')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__map_point

            from interface.srv import _map_point
            if _map_point.Metaclass_MapPoint_Request._TYPE_SUPPORT is None:
                _map_point.Metaclass_MapPoint_Request.__import_type_support__()
            if _map_point.Metaclass_MapPoint_Response._TYPE_SUPPORT is None:
                _map_point.Metaclass_MapPoint_Response.__import_type_support__()


class MapPoint(metaclass=Metaclass_MapPoint):
    from interface.srv._map_point import MapPoint_Request as Request
    from interface.srv._map_point import MapPoint_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
