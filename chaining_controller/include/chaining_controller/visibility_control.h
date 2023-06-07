#ifndef CHAINING_CONTROLLER__VISIBILITY_CHAINING_CONTROLLER_H_
#define CHAINING_CONTROLLER__VISIBILITY_CHAINING_CONTROLLER_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define CHAINING_CONTROLLER_EXPORT __attribute__((dllexport))
#define CHAINING_CONTROLLER_IMPORT __attribute__((dllimport))
#else
#define CHAINING_CONTROLLER_EXPORT __declspec(dllexport)
#define CHAINING_CONTROLLER_IMPORT __declspec(dllimport)
#endif
#ifdef CHAINING_CONTROLLER_BUILDING_DLL
#define CHAINING_CONTROLLER_PUBLIC CHAINING_CONTROLLER_EXPORT
#else
#define CHAINING_CONTROLLER_PUBLIC CHAINING_CONTROLLER_IMPORT
#endif
#define CHAINING_CONTROLLER_PUBLIC_TYPE CHAINING_CONTROLLER_PUBLIC
#define CHAINING_CONTROLLER_LOCAL
#else
#define CHAINING_CONTROLLER_EXPORT __attribute__((visibility("default")))
#define CHAINING_CONTROLLER_IMPORT
#if __GNUC__ >= 4
#define CHAINING_CONTROLLER_PUBLIC __attribute__((visibility("default")))
#define CHAINING_CONTROLLER_LOCAL __attribute__((visibility("hidden")))
#else
#define CHAINING_CONTROLLER_PUBLIC
#define CHAINING_CONTROLLER_LOCAL
#endif
#define CHAINING_CONTROLLER_PUBLIC_TYPE
#endif

#endif  // CHAINING_CONTROLLER__VISIBILITY_CHAINING_CONTROLLER_H_