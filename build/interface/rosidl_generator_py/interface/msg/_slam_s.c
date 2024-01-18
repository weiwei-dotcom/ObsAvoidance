// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from interface:msg/Slam.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "interface/msg/detail/slam__struct.h"
#include "interface/msg/detail/slam__functions.h"

ROSIDL_GENERATOR_C_IMPORT
<<<<<<< HEAD
bool sensor_msgs__msg__point_cloud2__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * sensor_msgs__msg__point_cloud2__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool geometry_msgs__msg__pose__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * geometry_msgs__msg__pose__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool geometry_msgs__msg__pose__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * geometry_msgs__msg__pose__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool sensor_msgs__msg__image__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * sensor_msgs__msg__image__convert_to_py(void * raw_ros_message);
=======
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool sensor_msgs__msg__point_cloud2__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * sensor_msgs__msg__point_cloud2__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool geometry_msgs__msg__pose_stamped__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * geometry_msgs__msg__pose_stamped__convert_to_py(void * raw_ros_message);
>>>>>>> 9ce6bb423e552849a267afd38d866d6092578e09

ROSIDL_GENERATOR_C_EXPORT
bool interface__msg__slam__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[25];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("interface.msg._slam.Slam", full_classname_dest, 24) == 0);
  }
  interface__msg__Slam * ros_message = _ros_message;
<<<<<<< HEAD
  {  // point_cloud
    PyObject * field = PyObject_GetAttrString(_pymsg, "point_cloud");
    if (!field) {
      return false;
    }
    if (!sensor_msgs__msg__point_cloud2__convert_from_py(field, &ros_message->point_cloud)) {
=======
  {  // header
    PyObject * field = PyObject_GetAttrString(_pymsg, "header");
    if (!field) {
      return false;
    }
    if (!std_msgs__msg__header__convert_from_py(field, &ros_message->header)) {
>>>>>>> 9ce6bb423e552849a267afd38d866d6092578e09
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
<<<<<<< HEAD
  {  // cam_pose
    PyObject * field = PyObject_GetAttrString(_pymsg, "cam_pose");
    if (!field) {
      return false;
    }
    if (!geometry_msgs__msg__pose__convert_from_py(field, &ros_message->cam_pose)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // world2cam
    PyObject * field = PyObject_GetAttrString(_pymsg, "world2cam");
    if (!field) {
      return false;
    }
    if (!geometry_msgs__msg__pose__convert_from_py(field, &ros_message->world2cam)) {
=======
  {  // point_cloud
    PyObject * field = PyObject_GetAttrString(_pymsg, "point_cloud");
    if (!field) {
      return false;
    }
    if (!sensor_msgs__msg__point_cloud2__convert_from_py(field, &ros_message->point_cloud)) {
>>>>>>> 9ce6bb423e552849a267afd38d866d6092578e09
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
<<<<<<< HEAD
  {  // img
    PyObject * field = PyObject_GetAttrString(_pymsg, "img");
    if (!field) {
      return false;
    }
    if (!sensor_msgs__msg__image__convert_from_py(field, &ros_message->img)) {
=======
  {  // transform_init2cur
    PyObject * field = PyObject_GetAttrString(_pymsg, "transform_init2cur");
    if (!field) {
      return false;
    }
    if (!geometry_msgs__msg__pose_stamped__convert_from_py(field, &ros_message->transform_init2cur)) {
>>>>>>> 9ce6bb423e552849a267afd38d866d6092578e09
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * interface__msg__slam__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Slam */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("interface.msg._slam");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Slam");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  interface__msg__Slam * ros_message = (interface__msg__Slam *)raw_ros_message;
<<<<<<< HEAD
  {  // point_cloud
    PyObject * field = NULL;
    field = sensor_msgs__msg__point_cloud2__convert_to_py(&ros_message->point_cloud);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "point_cloud", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // cam_pose
    PyObject * field = NULL;
    field = geometry_msgs__msg__pose__convert_to_py(&ros_message->cam_pose);
=======
  {  // header
    PyObject * field = NULL;
    field = std_msgs__msg__header__convert_to_py(&ros_message->header);
>>>>>>> 9ce6bb423e552849a267afd38d866d6092578e09
    if (!field) {
      return NULL;
    }
    {
<<<<<<< HEAD
      int rc = PyObject_SetAttrString(_pymessage, "cam_pose", field);
=======
      int rc = PyObject_SetAttrString(_pymessage, "header", field);
>>>>>>> 9ce6bb423e552849a267afd38d866d6092578e09
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
<<<<<<< HEAD
  {  // world2cam
    PyObject * field = NULL;
    field = geometry_msgs__msg__pose__convert_to_py(&ros_message->world2cam);
=======
  {  // point_cloud
    PyObject * field = NULL;
    field = sensor_msgs__msg__point_cloud2__convert_to_py(&ros_message->point_cloud);
>>>>>>> 9ce6bb423e552849a267afd38d866d6092578e09
    if (!field) {
      return NULL;
    }
    {
<<<<<<< HEAD
      int rc = PyObject_SetAttrString(_pymessage, "world2cam", field);
=======
      int rc = PyObject_SetAttrString(_pymessage, "point_cloud", field);
>>>>>>> 9ce6bb423e552849a267afd38d866d6092578e09
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
<<<<<<< HEAD
  {  // img
    PyObject * field = NULL;
    field = sensor_msgs__msg__image__convert_to_py(&ros_message->img);
=======
  {  // transform_init2cur
    PyObject * field = NULL;
    field = geometry_msgs__msg__pose_stamped__convert_to_py(&ros_message->transform_init2cur);
>>>>>>> 9ce6bb423e552849a267afd38d866d6092578e09
    if (!field) {
      return NULL;
    }
    {
<<<<<<< HEAD
      int rc = PyObject_SetAttrString(_pymessage, "img", field);
=======
      int rc = PyObject_SetAttrString(_pymessage, "transform_init2cur", field);
>>>>>>> 9ce6bb423e552849a267afd38d866d6092578e09
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
