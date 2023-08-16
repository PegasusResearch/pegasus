#ifndef AUTOPILOT_MODES__VISIBILITY_CONTROL_H_
#define AUTOPILOT_MODES__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define AUTOPILOT_MODES_EXPORT __attribute__ ((dllexport))
    #define AUTOPILOT_MODES_IMPORT __attribute__ ((dllimport))
  #else
    #define AUTOPILOT_MODES_EXPORT __declspec(dllexport)
    #define AUTOPILOT_MODES_IMPORT __declspec(dllimport)
  #endif
  #ifdef AUTOPILOT_MODES_BUILDING_LIBRARY
    #define AUTOPILOT_MODES_PUBLIC AUTOPILOT_MODES_EXPORT
  #else
    #define AUTOPILOT_MODES_PUBLIC AUTOPILOT_MODES_IMPORT
  #endif
  #define AUTOPILOT_MODES_PUBLIC_TYPE AUTOPILOT_MODES_PUBLIC
  #define AUTOPILOT_MODES_LOCAL
#else
  #define AUTOPILOT_MODES_EXPORT __attribute__ ((visibility("default")))
  #define AUTOPILOT_MODES_IMPORT
  #if __GNUC__ >= 4
    #define AUTOPILOT_MODES_PUBLIC __attribute__ ((visibility("default")))
    #define AUTOPILOT_MODES_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define AUTOPILOT_MODES_PUBLIC
    #define AUTOPILOT_MODES_LOCAL
  #endif
  #define AUTOPILOT_MODES_PUBLIC_TYPE
#endif

#endif  // AUTOPILOT_MODES__VISIBILITY_CONTROL_H_
