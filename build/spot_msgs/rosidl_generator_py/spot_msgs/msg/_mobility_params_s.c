// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from spot_msgs:msg/MobilityParams.idl
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
#include "spot_msgs/msg/detail/mobility_params__struct.h"
#include "spot_msgs/msg/detail/mobility_params__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool geometry_msgs__msg__pose__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * geometry_msgs__msg__pose__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool spot_msgs__msg__mobility_params__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[46];
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
    assert(strncmp("spot_msgs.msg._mobility_params.MobilityParams", full_classname_dest, 45) == 0);
  }
  spot_msgs__msg__MobilityParams * ros_message = _ros_message;
  {  // body_control
    PyObject * field = PyObject_GetAttrString(_pymsg, "body_control");
    if (!field) {
      return false;
    }
    if (!geometry_msgs__msg__pose__convert_from_py(field, &ros_message->body_control)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // locomotion_hint
    PyObject * field = PyObject_GetAttrString(_pymsg, "locomotion_hint");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->locomotion_hint = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // stair_hint
    PyObject * field = PyObject_GetAttrString(_pymsg, "stair_hint");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->stair_hint = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * spot_msgs__msg__mobility_params__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of MobilityParams */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("spot_msgs.msg._mobility_params");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "MobilityParams");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  spot_msgs__msg__MobilityParams * ros_message = (spot_msgs__msg__MobilityParams *)raw_ros_message;
  {  // body_control
    PyObject * field = NULL;
    field = geometry_msgs__msg__pose__convert_to_py(&ros_message->body_control);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "body_control", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // locomotion_hint
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->locomotion_hint);
    {
      int rc = PyObject_SetAttrString(_pymessage, "locomotion_hint", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // stair_hint
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->stair_hint ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "stair_hint", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
