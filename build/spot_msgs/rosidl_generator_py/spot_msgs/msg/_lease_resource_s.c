// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from spot_msgs:msg/LeaseResource.idl
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
#include "spot_msgs/msg/detail/lease_resource__struct.h"
#include "spot_msgs/msg/detail/lease_resource__functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"

bool spot_msgs__msg__lease__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * spot_msgs__msg__lease__convert_to_py(void * raw_ros_message);
bool spot_msgs__msg__lease_owner__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * spot_msgs__msg__lease_owner__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool spot_msgs__msg__lease_resource__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[44];
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
    assert(strncmp("spot_msgs.msg._lease_resource.LeaseResource", full_classname_dest, 43) == 0);
  }
  spot_msgs__msg__LeaseResource * ros_message = _ros_message;
  {  // resource
    PyObject * field = PyObject_GetAttrString(_pymsg, "resource");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->resource, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // lease
    PyObject * field = PyObject_GetAttrString(_pymsg, "lease");
    if (!field) {
      return false;
    }
    if (!spot_msgs__msg__lease__convert_from_py(field, &ros_message->lease)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // lease_owner
    PyObject * field = PyObject_GetAttrString(_pymsg, "lease_owner");
    if (!field) {
      return false;
    }
    if (!spot_msgs__msg__lease_owner__convert_from_py(field, &ros_message->lease_owner)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * spot_msgs__msg__lease_resource__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of LeaseResource */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("spot_msgs.msg._lease_resource");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "LeaseResource");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  spot_msgs__msg__LeaseResource * ros_message = (spot_msgs__msg__LeaseResource *)raw_ros_message;
  {  // resource
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->resource.data,
      strlen(ros_message->resource.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "resource", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // lease
    PyObject * field = NULL;
    field = spot_msgs__msg__lease__convert_to_py(&ros_message->lease);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "lease", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // lease_owner
    PyObject * field = NULL;
    field = spot_msgs__msg__lease_owner__convert_to_py(&ros_message->lease_owner);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "lease_owner", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
