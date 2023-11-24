#ifndef CB_HW_FW_POS_READ_POS_VEL__VISIBILITY_CONTROL_H_
#define CB_HW_FW_POS_READ_POS_VEL__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define CB_HW_FW_POS_READ_POS_VEL_EXPORT __attribute__ ((dllexport))
    #define CB_HW_FW_POS_READ_POS_VEL_IMPORT __attribute__ ((dllimport))
  #else
    #define CB_HW_FW_POS_READ_POS_VEL_EXPORT __declspec(dllexport)
    #define CB_HW_FW_POS_READ_POS_VEL_IMPORT __declspec(dllimport)
  #endif
  #ifdef CB_HW_FW_POS_READ_POS_VEL_BUILDING_LIBRARY
    #define CB_HW_FW_POS_READ_POS_VEL_PUBLIC CB_HW_FW_POS_READ_POS_VEL_EXPORT
  #else
    #define CB_HW_FW_POS_READ_POS_VEL_PUBLIC CB_HW_FW_POS_READ_POS_VEL_IMPORT
  #endif
  #define CB_HW_FW_POS_READ_POS_VEL_PUBLIC_TYPE CB_HW_FW_POS_READ_POS_VEL_PUBLIC
  #define CB_HW_FW_POS_READ_POS_VEL_LOCAL
#else
  #define CB_HW_FW_POS_READ_POS_VEL_EXPORT __attribute__ ((visibility("default")))
  #define CB_HW_FW_POS_READ_POS_VEL_IMPORT
  #if __GNUC__ >= 4
    #define CB_HW_FW_POS_READ_POS_VEL_PUBLIC __attribute__ ((visibility("default")))
    #define CB_HW_FW_POS_READ_POS_VEL_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define CB_HW_FW_POS_READ_POS_VEL_PUBLIC
    #define CB_HW_FW_POS_READ_POS_VEL_LOCAL
  #endif
  #define CB_HW_FW_POS_READ_POS_VEL_PUBLIC_TYPE
#endif

#endif  // CB_HW_FW_POS_READ_POS_VEL__VISIBILITY_CONTROL_H_
