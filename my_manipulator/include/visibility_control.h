#ifndef MANIPULATOR__VISIBILITY_CONTROL_H_
#define MANIPULATOR__VISIBILITY_CONTROL_H_

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define MANIPULATOR_EXPORT __attribute__((dllexport))
    #define MANIPULATOR_IMPORT __attribute__((dllimport))
  #else
    #define MANIPULATOR_EXPORT __declspec(dllexport)
    #define MANIPULATOR_IMPORT __declspec(dllimport)
  #endif
  #ifdef MANIPULATOR_BUILDING_DLL
    #define MANIPULATOR_PUBLIC MANIPULATOR_EXPORT
  #else
    #define MANIPULATOR_PUBLIC MANIPULATOR_IMPORT
  #endif
  #define MANIPULATOR_LOCAL
#else
  #define MANIPULATOR_EXPORT __attribute__((visibility("default")))
  #define MANIPULATOR_IMPORT
  #if __GNUC__ >= 4
    #define MANIPULATOR_PUBLIC __attribute__((visibility("default")))
    #define MANIPULATOR_LOCAL __attribute__((visibility("hidden")))
  #else
    #define MANIPULATOR_PUBLIC
    #define MANIPULATOR_LOCAL
  #endif
#endif

#endif  // MANIPULATOR__VISIBILITY_CONTROL_H_
