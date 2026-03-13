#pragma once
#include <cstdio>
#include <cmath>
#include <cstdlib>

static int g_pass = 0, g_fail = 0;

#define CHECK(cond, msg) do { \
    if (cond) { g_pass++; } \
    else { g_fail++; std::fprintf(stderr, "FAIL [%s:%d]: %s\n", __FILE__, __LINE__, msg); } \
} while(0)

#define APPROX(a, b, tol) (std::abs((a)-(b)) < (tol))

static int reportResults() {
    std::printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail > 0 ? 1 : 0;
}
