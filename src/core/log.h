#pragma once
// Logging goes through spdlog. These macros keep call sites short and let us
// swap sinks (file, Android logcat) in one place later.
// Format strings use std::format syntax: LOG_INFO("window {}x{}", w, h).

#include <spdlog/spdlog.h>

#define LOG_INFO(...) ::spdlog::info(__VA_ARGS__)
#define LOG_WARN(...) ::spdlog::warn(__VA_ARGS__)
#define LOG_ERROR(...) ::spdlog::error(__VA_ARGS__)
