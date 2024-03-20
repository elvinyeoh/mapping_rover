
#ifndef DIFFDRIVE_ROVER__VISIBILITY_CONTROL_H_
#define DIFFDRIVE_ROVER__VISIBILITY_CONTROL_H_

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define DIFFDRIVE_ROVER_EXPORT __attribute__((dllexport))
#define DIFFDRIVE_ROVER_IMPORT __attribute__((dllimport))
#else
#define DIFFDRIVE_ROVER_EXPORT __declspec(dllexport)
#define DIFFDRIVE_ROVER_IMPORT __declspec(dllimport)
#endif
#ifdef DIFFDRIVE_ROVER_BUILDING_DLL
#define DIFFDRIVE_ROVER_PUBLIC DIFFDRIVE_ROVER_EXPORT
#else
#define DIFFDRIVE_ROVER_PUBLIC DIFFDRIVE_ROVER_IMPORT
#endif
#define DIFFDRIVE_ROVER_PUBLIC_TYPE DIFFDRIVE_ROVER_PUBLIC
#define DIFFDRIVE_ROVER_LOCAL
#else
#define DIFFDRIVE_ROVER_EXPORT __attribute__((visibility("default")))
#define DIFFDRIVE_ROVER_IMPORT
#if __GNUC__ >= 4
#define DIFFDRIVE_ROVER_PUBLIC __attribute__((visibility("default")))
#define DIFFDRIVE_ROVER_LOCAL __attribute__((visibility("hidden")))
#else
#define DIFFDRIVE_ROVER_PUBLIC
#define DIFFDRIVE_ROVER_LOCAL
#endif
#define DIFFDRIVE_ROVER_PUBLIC_TYPE
#endif

#endif  // DIFFDRIVE_ROVER__VISIBILITY_CONTROL_H_
