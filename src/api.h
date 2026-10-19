#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define G1GravityCompensationController_DLLIMPORT __declspec(dllimport)
#  define G1GravityCompensationController_DLLEXPORT __declspec(dllexport)
#  define G1GravityCompensationController_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define G1GravityCompensationController_DLLIMPORT __attribute__((visibility("default")))
#    define G1GravityCompensationController_DLLEXPORT __attribute__((visibility("default")))
#    define G1GravityCompensationController_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define G1GravityCompensationController_DLLIMPORT
#    define G1GravityCompensationController_DLLEXPORT
#    define G1GravityCompensationController_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef G1GravityCompensationController_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define G1GravityCompensationController_DLLAPI
#  define G1GravityCompensationController_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef G1GravityCompensationController_EXPORTS
#    define G1GravityCompensationController_DLLAPI G1GravityCompensationController_DLLEXPORT
#  else
#    define G1GravityCompensationController_DLLAPI G1GravityCompensationController_DLLIMPORT
#  endif // G1GravityCompensationController_EXPORTS
#  define G1GravityCompensationController_LOCAL G1GravityCompensationController_DLLLOCAL
#endif // G1GravityCompensationController_STATIC
