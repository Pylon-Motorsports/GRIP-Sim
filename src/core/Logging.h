#pragma once
#include <cstdio>
#include <cstdlib>

#define LOG_INFO(fmt, ...)  std::fprintf(stdout, "[INFO]  " fmt "\n", ##__VA_ARGS__)
#define LOG_WARN(fmt, ...)  std::fprintf(stderr, "[WARN]  " fmt "\n", ##__VA_ARGS__)
#define LOG_ERROR(fmt, ...) std::fprintf(stderr, "[ERROR] " fmt "\n", ##__VA_ARGS__)

#ifdef NDEBUG
#  define LOG_DEBUG(fmt, ...) (void)0
#else
#  define LOG_DEBUG(fmt, ...) std::fprintf(stdout, "[DEBUG] " fmt "\n", ##__VA_ARGS__)
#endif

#define GRIP_ASSERT(cond, msg) \
    do { if (!(cond)) { LOG_ERROR("Assertion failed: %s", msg); std::abort(); } } while(0)
