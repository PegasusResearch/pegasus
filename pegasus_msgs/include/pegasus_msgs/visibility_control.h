#ifndef PEGASUS_MSGS_CPP__VISIBILITY_CONTROL_H_
#define PEGASUS_MSGS_CPP__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define PEGASUS_MSGS_CPP_EXPORT __attribute__ ((dllexport))
    #define PEGASUS_MSGS_CPP_IMPORT __attribute__ ((dllimport))
  #else
    #define PEGASUS_MSGS_CPP_EXPORT __declspec(dllexport)
    #define PEGASUS_MSGS_CPP_IMPORT __declspec(dllimport)
  #endif
  #ifdef PEGASUS_MSGS_CPP_BUILDING_DLL
    #define PEGASUS_MSGS_CPP_PUBLIC PEGASUS_MSGS_CPP_EXPORT
  #else
    #define PEGASUS_MSGS_CPP_PUBLIC PEGASUS_MSGS_CPP_IMPORT
  #endif
  #define PEGASUS_MSGS_CPP_PUBLIC_TYPE PEGASUS_MSGS_CPP_PUBLIC
  #define PEGASUS_MSGS_CPP_LOCAL
#else
  #define PEGASUS_MSGS_CPP_EXPORT __attribute__ ((visibility("default")))
  #define PEGASUS_MSGS_CPP_IMPORT
  #if __GNUC__ >= 4
    #define PEGASUS_MSGS_CPP_PUBLIC __attribute__ ((visibility("default")))
    #define PEGASUS_MSGS_CPP_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define PEGASUS_MSGS_CPP_PUBLIC
    #define PEGASUS_MSGS_CPP_LOCAL
  #endif
  #define PEGASUS_MSGS_CPP_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // PEGASUS_MSGS_CPP__VISIBILITY_CONTROL_H_