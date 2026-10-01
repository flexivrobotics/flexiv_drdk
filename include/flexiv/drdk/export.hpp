/**
 * @file export.hpp
 * @brief Export macro for the public flexiv::drdk API.
 * @copyright Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved.
 */
#ifndef FLEXIV_DRDK_EXPORT_HPP_
#define FLEXIV_DRDK_EXPORT_HPP_

// Marks the public flexiv::drdk API for export from the library. The library is built with hidden
// default visibility so that its statically embedded dependencies stay private, so every public
// class and free function must be tagged with this macro to remain visible to user applications.
#if defined(_WIN32)
// Windows: export the API from the DLL when building it, import it otherwise.
#if defined(FLEXIV_DRDK_BUILDING)
#define DRDK_API __declspec(dllexport)
#else
#define DRDK_API __declspec(dllimport)
#endif
#else
// Linux/macOS: the library is compiled with -fvisibility=hidden, so the API is marked explicitly.
#define DRDK_API __attribute__((visibility("default")))
#endif

#endif /* FLEXIV_DRDK_EXPORT_HPP_ */
