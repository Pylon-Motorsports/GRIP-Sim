#include "test_common.hpp"
#include "../src/Engine.hpp"

// ============================================================================
// Torque curve tests
// ============================================================================

static void testTorqueCurveLookup()
{
    TorqueCurve curve({
        {1000.f, 30.f},
        {3500.f, 80.f},
        {6500.f, 45.f},
    });

    CHECK(APPROX(curve.lookup(1000.f), 30.f, 0.01f), "torque at 1000 RPM");
    CHECK(APPROX(curve.lookup(6500.f), 45.f, 0.01f), "torque at 6500 RPM");

    float mid = curve.lookup(2250.f);
    CHECK(APPROX(mid, 55.f, 0.01f), "torque interpolation at 2250 RPM");

    CHECK(APPROX(curve.lookup(500.f), 30.f, 0.01f), "torque below min RPM");
    CHECK(APPROX(curve.lookup(7000.f), 45.f, 0.01f), "torque above max RPM");
}

// ============================================================================
// Engine tests
// ============================================================================

static void testEngineIdleWithNoThrottle()
{
    Engine eng;
    eng.torqueCurve = TorqueCurve({
        {1000.f, 30.f}, {3500.f, 80.f}, {6500.f, 45.f},
    });
    eng.rpm = 1200.f;

    for (int i = 0; i < 1000; ++i)
        eng.update(0.f, 0.f, 1.f/120.f);

    CHECK(APPROX(eng.rpm, eng.idleRpm, 50.f), "engine stays near idle with no throttle");
}

static void testEngineRevsUpWithThrottle()
{
    Engine eng;
    eng.torqueCurve = TorqueCurve({
        {1000.f, 30.f}, {3500.f, 80.f}, {6500.f, 45.f},
    });
    eng.rpm = 1200.f;

    for (int i = 0; i < 240; ++i)
        eng.update(1.f, true, 0.f, 1.f/120.f);

    CHECK(eng.rpm > 2000.f, "engine revs up with throttle (clutch disengaged)");
    CHECK(eng.rpm <= eng.rpmLimit, "engine respects RPM limit");
}

static void testEngineRevLimiter()
{
    Engine eng;
    eng.torqueCurve = TorqueCurve({
        {1000.f, 30.f}, {6500.f, 80.f},
    });
    eng.rpm = 6400.f;

    for (int i = 0; i < 600; ++i)
        eng.update(1.f, 0.f, 1.f/120.f);

    CHECK(eng.rpm <= eng.rpmLimit, "rev limiter caps RPM");
}

static void testClutchEngagement()
{
    Engine eng;
    eng.torqueCurve = TorqueCurve({
        {1000.f, 30.f}, {3500.f, 80.f}, {6500.f, 45.f},
    });
    eng.rpm = 1200.f;

    float torque = eng.update(1.f, 0.f, 1.f/120.f);
    CHECK(APPROX(torque, 0.f, 1.f), "no torque below clutch engage RPM");

    eng.rpm = 3000.f;
    torque = eng.update(1.f, 0.f, 1.f/120.f);
    CHECK(torque > 10.f, "torque output when clutch engaged");
}

int main()
{
    std::printf("Running engine tests...\n\n");

    testTorqueCurveLookup();
    testEngineIdleWithNoThrottle();
    testEngineRevsUpWithThrottle();
    testEngineRevLimiter();
    testClutchEngagement();

    return reportResults();
}
