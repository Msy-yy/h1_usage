// visibility_control.h
#ifndef h1_arm_control_hardware_VISIBILITY_CONTROL_H_
#define h1_arm_control_hardware_VISIBILITY_CONTROL_H_

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define h1_arm_control_hardware_EXPORT __attribute__ ((dllexport))
    #define h1_arm_control_hardware_IMPORT __attribute__ ((dllimport))
  #else
    #define h1_arm_control_hardware_EXPORT __declspec(dllexport)
    #define h1_arm_control_hardware_IMPORT __declspec(dllimport)
  #endif
  #ifdef h1_arm_control_hardware_BUILDING_DLL
    #define h1_arm_control_hardware_PUBLIC h1_arm_control_hardware_EXPORT
  #else
    #define h1_arm_control_hardware_PUBLIC h1_arm_control_hardware_IMPORT
  #endif
  #define h1_arm_control_hardware_PUBLIC_TYPE h1_arm_control_hardware_PUBLIC
  #define h1_arm_control_hardware_LOCAL
#else
  #define h1_arm_control_hardware_EXPORT __attribute__ ((visibility("default")))
  #define h1_arm_control_hardware_IMPORT
  #if __GNUC__ >= 4
    #define h1_arm_control_hardware_PUBLIC __attribute__ ((visibility("default")))
    #define h1_arm_control_hardware_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define h1_arm_control_hardware_PUBLIC
    #define h1_arm_control_hardware_LOCAL
  #endif
  #define h1_arm_control_hardware_PUBLIC_TYPE
#endif

#endif  // h1_arm_control_hardware_VISIBILITY_CONTROL_H_