// Copyright (c) 2025
#pragma once

#if defined(_WIN32)
#if defined(NTP_SERVER_BUILDING_DLL)
#define NTP_SERVER_API __declspec(dllexport)
#elif defined(NTP_SERVER_SHARED)
#define NTP_SERVER_API __declspec(dllimport)
#else
#define NTP_SERVER_API
#endif
#else
#define NTP_SERVER_API
#endif
