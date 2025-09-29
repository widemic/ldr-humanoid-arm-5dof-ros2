#ifndef HUMANOID_ARM_CONTROL__VISIBILITY_CONTROL_H_
#define HUMANOID_ARM_CONTROL__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define TEMPLATES__ROS2_CONTROL__EXPORT __attribute__ ((dllexport))
    #define TEMPLATES__ROS2_CONTROL__IMPORT __attribute__ ((dllimport))
  #else
    #define TEMPLATES__ROS2_CONTROL__EXPORT __declspec(dllexport)
    #define TEMPLATES__ROS2_CONTROL__IMPORT __declspec(dllimport)
  #endif
  #ifdef HUMANOID_ARM_CONTROL_BUILDING_DLL
    #define TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC TEMPLATES__ROS2_CONTROL__EXPORT
  #else
    #define TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC TEMPLATES__ROS2_CONTROL__IMPORT
  #endif
  #define TEMPLATES__ROS2_CONTROL__VISIBILITY_LOCAL
#else
  #define TEMPLATES__ROS2_CONTROL__EXPORT __attribute__ ((visibility("default")))
  #define TEMPLATES__ROS2_CONTROL__IMPORT
  #if __GNUC__ >= 4
    #define TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC __attribute__ ((visibility("default")))
    #define TEMPLATES__ROS2_CONTROL__VISIBILITY_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
    #define TEMPLATES__ROS2_CONTROL__VISIBILITY_LOCAL
  #endif
#endif

#endif  // HUMANOID_ARM_CONTROL__VISIBILITY_CONTROL_H_