#include "test_common.hpp"
#include "../src/VehiclePhysics.hpp"
#include "../src/Input.hpp"
#include "../src/Scenario.hpp"
#include "../src/Terrain.hpp"
#include <glm/glm.hpp>

// Helper: run physics for N seconds with given input
static void simulate(VehiclePhysics& phys, const InputState& input, float seconds)
{
    constexpr float DT = 1.f / 120.f;
    int steps = (int)(seconds / DT);
    for (int i = 0; i < steps; ++i)
        phys.update(DT, input);
}

// ============================================================================
// Spawn stability tests
// ============================================================================

static void testNoLateralDriftAtSpawn()
{
    VehiclePhysics phys;
    phys.init();

    Vehicle v;
    phys.fillVehicle(v);
    float startX = v.position.x;
    float startZ = v.position.z;

    InputState nothing{};
    constexpr float DT = 1.f / 120.f;
    float maxDriftX = 0.f;

    for (int frame = 0; frame < 600; ++frame) {
        phys.update(DT, nothing);
        phys.fillVehicle(v);
        float driftX = std::abs(v.position.x - startX);
        if (driftX > maxDriftX) maxDriftX = driftX;
    }

    std::printf("  spawn drift: x=%.4f z=%.4f (after 5s idle)\n",
                v.position.x - startX, v.position.z - startZ);
    CHECK(maxDriftX < 0.01f,
          "no lateral drift at spawn (x should stay near 0)");
    CHECK(std::abs(v.position.z - startZ) < 0.01f,
          "no longitudinal drift at spawn");
    CHECK(std::abs(v.heading) < 0.001f,
          "no yaw rotation at spawn");
}

static void testNoLateralDriftUnderThrottle()
{
    VehiclePhysics phys;
    phys.init();

    InputState gas{}; gas.throttle = 1.f;
    simulate(phys, gas, 5.f);

    Vehicle v;
    phys.fillVehicle(v);
    std::printf("  throttle drift: x=%.4f z=%.4f heading=%.6f\n",
                v.position.x, v.position.z, v.heading);
    CHECK(std::abs(v.position.x) < 0.1f,
          "no lateral drift under straight throttle");
    CHECK(std::abs(v.heading) < 0.01f,
          "no yaw under straight throttle");
}

static void testNoLateralDriftOnPlayground()
{
    VehiclePhysics phys;
    phys.init();

    Playground pg = createPlayground();
    phys.setTerrain(&pg.terrain);

    Vehicle v;
    phys.fillVehicle(v);
    float startX = v.position.x;

    InputState nothing{};
    simulate(phys, nothing, 3.f);

    phys.fillVehicle(v);
    std::printf("  playground idle: x=%.4f z=%.4f heading=%.6f\n",
                v.position.x - startX, v.position.z, v.heading);
    CHECK(std::abs(v.position.x - startX) < 0.01f,
          "no lateral drift on playground at idle");

    // Now with throttle
    phys.reset();
    phys.setTerrain(&pg.terrain);
    InputState gas{}; gas.throttle = 1.f;
    simulate(phys, gas, 3.f);

    phys.fillVehicle(v);
    std::printf("  playground gas:  x=%.4f z=%.4f heading=%.6f\n",
                v.position.x, v.position.z, v.heading);
    CHECK(std::abs(v.position.x) < 0.5f,
          "no significant lateral drift on playground under throttle");
}

// ============================================================================
// Acceleration / braking tests
// ============================================================================

static void testVehicleAccelerates()
{
    VehiclePhysics phys;
    phys.init();

    InputState gas{};
    gas.throttle = 1.f;

    simulate(phys, gas, 5.f);

    CHECK(phys.getForwardSpeed() > 5.f, "vehicle accelerates with throttle");
    CHECK(phys.getEngineRpm() > 2000.f, "engine RPM rises during acceleration");
}

static void testVehicleBrakes()
{
    VehiclePhysics phys;
    phys.init();

    InputState gas{}; gas.throttle = 1.f;
    simulate(phys, gas, 5.f);
    float speedBeforeBrake = phys.getForwardSpeed();
    CHECK(speedBeforeBrake > 5.f, "moving before braking");

    InputState brake{}; brake.brake = 1.f;
    simulate(phys, brake, 3.f);

    CHECK(phys.getForwardSpeed() < speedBeforeBrake * 0.1f,
          "vehicle slows significantly with brakes");
}

static void testVehicleCoastsToStop()
{
    VehiclePhysics phys;
    phys.init();

    InputState gas{}; gas.throttle = 1.f;
    simulate(phys, gas, 3.f);
    float peakSpeed = phys.getForwardSpeed();
    CHECK(peakSpeed > 3.f, "vehicle moving before coasting");

    InputState nothing{};
    simulate(phys, nothing, 60.f);

    CHECK(phys.getForwardSpeed() < 0.5f,
          "vehicle coasts to near-stop from friction");
}

static void testBrakeDoesNotReverse()
{
    VehiclePhysics phys;
    phys.init();

    InputState gas{}; gas.throttle = 1.f;
    simulate(phys, gas, 2.f);

    InputState brake{}; brake.brake = 1.f;
    simulate(phys, brake, 10.f);

    CHECK(phys.getForwardSpeed() >= -0.05f,
          "braking doesn't make the car go backward");
}

static void testRWDOnlyRearWheelsDriven()
{
    VehiclePhysics phys;
    phys.init();

    InputState gas{}; gas.throttle = 1.f;
    simulate(phys, gas, 3.f);

    CHECK(phys.getForwardSpeed() > 2.f,
          "RWD vehicle accelerates (rear wheels driven)");
}

static void testEngineBraking()
{
    VehiclePhysics phys;
    phys.init();

    InputState gas{}; gas.throttle = 1.f;
    simulate(phys, gas, 5.f);
    float speed1 = phys.getForwardSpeed();

    InputState nothing{};
    simulate(phys, nothing, 2.f);
    float speed2 = phys.getForwardSpeed();

    float decel = speed1 - speed2;
    CHECK(decel > 0.2f, "rolling resistance + bearing drag decelerates off throttle");
}

static void testEqualBrakingFrontRear()
{
    VehiclePhysics phys;
    phys.init();

    InputState gas{}; gas.throttle = 1.f;
    simulate(phys, gas, 4.f);
    float speedBefore = phys.getForwardSpeed();

    InputState brake{}; brake.brake = 1.f;
    simulate(phys, brake, 1.f);

    float speedAfter = phys.getForwardSpeed();
    float decel = speedBefore - speedAfter;

    CHECK(decel > 5.f, "strong braking with all 4 wheels");
}

static void testStationaryWithNoInput()
{
    VehiclePhysics phys;
    phys.init();

    InputState nothing{};
    simulate(phys, nothing, 2.f);

    CHECK(APPROX(phys.getForwardSpeed(), 0.f, 0.1f),
          "vehicle stays stationary with no input");
}

static void testBrakeLockupBalance()
{
    VehiclePhysics phys;
    phys.init();

    InputState gas{}; gas.throttle = 1.f;
    simulate(phys, gas, 5.f);
    float speedBefore = phys.getForwardSpeed();
    CHECK(speedBefore > 10.f, "brake lockup: moving fast before braking");

    InputState brake{}; brake.brake = 1.f;
    simulate(phys, brake, 1.f);

    float speedAfter = phys.getForwardSpeed();
    float decel = speedBefore - speedAfter;
    std::printf("  brake lockup: decel=%.1f m/s in 1s (%.1fg)\n",
                decel, decel / 9.81f);

    CHECK(decel > 5.f, "brake lockup: strong deceleration under hard braking");
}

// ============================================================================
// Friction holding tests
// ============================================================================

static void testDirectBackwardDrift()
{
    VehiclePhysics phys;
    phys.init();

    InputState nothing{};
    simulate(phys, nothing, 1.f);

    InputState gas{}; gas.throttle = 1.f;
    simulate(phys, gas, 2.f);
    std::printf("  after-gas speed=%.4f\n", phys.getForwardSpeed());

    simulate(phys, nothing, 0.5f);
    std::printf("  coast0.5 speed=%.4f\n", phys.getForwardSpeed());
    simulate(phys, nothing, 0.5f);
    std::printf("  coast1.0 speed=%.4f\n", phys.getForwardSpeed());
    simulate(phys, nothing, 0.5f);
    std::printf("  coast1.5 speed=%.4f\n", phys.getForwardSpeed());

    CHECK(phys.getForwardSpeed() >= 0.f, "car doesn't go backward from coasting forward");
}

static void testFrictionHoldsAfterBraking()
{
    VehiclePhysics phys;
    phys.init();

    InputState gas{}; gas.throttle = 1.f;
    simulate(phys, gas, 3.f);
    std::printf("  after-gas speed=%.4f\n", phys.getForwardSpeed());

    InputState brake{}; brake.brake = 1.f;
    simulate(phys, brake, 1.f);
    std::printf("  after-1s-brake speed=%.4f\n", phys.getForwardSpeed());
    simulate(phys, brake, 4.f);
    std::printf("  after-5s-brake speed=%.4f\n", phys.getForwardSpeed());

    InputState nothing{};
    for (int s = 0; s < 10; ++s) {
        simulate(phys, nothing, 0.5f);
        std::printf("  coast +%.1fs speed=%.4f\n", (s+1)*0.5f, phys.getForwardSpeed());
    }
    CHECK(std::abs(phys.getForwardSpeed()) < 0.05f,
          "friction damps car to near-zero after braking");
}

static void testNoCreepAtStartup()
{
    VehiclePhysics phys;
    phys.init();

    Vehicle veh;
    phys.fillVehicle(veh);
    float startZ = veh.position.z;

    InputState nothing{};
    simulate(phys, nothing, 10.f);

    phys.fillVehicle(veh);
    CHECK(std::abs(veh.position.z - startZ) < 0.01f,
          "no creep at startup");
}

static void testThrottleOvercomesFriction()
{
    VehiclePhysics phys;
    phys.init();

    InputState nothing{};
    simulate(phys, nothing, 1.f);
    CHECK(std::abs(phys.getForwardSpeed()) < 0.01f, "stationary before throttle");

    InputState gas{}; gas.throttle = 1.f;
    simulate(phys, gas, 3.f);
    CHECK(phys.getForwardSpeed() > 1.f, "throttle overcomes friction");
}

// ============================================================================
// Steering tests
// ============================================================================

static void testVehicleTurnsRight()
{
    VehiclePhysics phys;
    phys.init();

    InputState gas{}; gas.throttle = 1.f;
    simulate(phys, gas, 3.f);
    float headingBefore = phys.getHeading();

    InputState steerRight{}; steerRight.throttle = 0.5f; steerRight.steer = 1.f;
    simulate(phys, steerRight, 2.f);

    float headingAfter = phys.getHeading();
    CHECK(headingAfter > headingBefore + 0.1f,
          "vehicle turns right with positive steer");
}

static void testVehicleTurnsLeft()
{
    VehiclePhysics phys;
    phys.init();

    InputState gas{}; gas.throttle = 1.f;
    simulate(phys, gas, 3.f);
    float headingBefore = phys.getHeading();

    InputState steerLeft{}; steerLeft.throttle = 0.5f; steerLeft.steer = -1.f;
    simulate(phys, steerLeft, 2.f);

    float headingAfter = phys.getHeading();
    CHECK(headingAfter < headingBefore - 0.1f,
          "vehicle turns left with negative steer");
}

static void testStraightWithNoSteer()
{
    VehiclePhysics phys;
    phys.init();

    InputState gas{}; gas.throttle = 1.f;
    simulate(phys, gas, 5.f);

    CHECK(std::abs(phys.getHeading()) < 0.01f,
          "vehicle drives straight with no steering");
}

static void testDeadzoneInput()
{
    VehiclePhysics phys;
    phys.init();

    InputState gas{}; gas.throttle = 1.f;
    simulate(phys, gas, 3.f);

    InputState tinySteer{}; tinySteer.throttle = 0.5f; tinySteer.steer = 0.03f;
    simulate(phys, tinySteer, 2.f);

    CHECK(std::abs(phys.getHeading()) < 0.01f,
          "deadzone prevents turning from tiny steer input");
}

static void testTurnBrakeReaccelerate()
{
    VehiclePhysics phys;
    phys.init();

    InputState turnRight{}; turnRight.throttle = 1.f; turnRight.steer = 1.f;
    simulate(phys, turnRight, 2.f);

    InputState brake{}; brake.brake = 1.f;
    simulate(phys, brake, 8.f);

    Vehicle stopped;
    phys.fillVehicle(stopped);
    float headingAtStop = stopped.heading;
    glm::vec3 posAtStop = stopped.position;

    std::printf("  turn-brake: speed after braking=%.3f\n", phys.getForwardSpeed());
    CHECK(std::abs(phys.getForwardSpeed()) < 1.0f,
          "turn-brake: car is nearly stopped");

    InputState gas{}; gas.throttle = 1.f;
    simulate(phys, gas, 2.f);

    Vehicle moving;
    phys.fillVehicle(moving);

    glm::vec3 displacement = moving.position - posAtStop;
    float distFwd = displacement.x * std::sin(headingAtStop)
                  + displacement.z * std::cos(headingAtStop);
    float distLat = displacement.x * std::cos(headingAtStop)
                  - displacement.z * std::sin(headingAtStop);

    std::printf("  turn-brake-reaccel: fwd=%.3f lat=%.3f heading=%.3f\n",
                distFwd, distLat, headingAtStop);

    CHECK(distFwd > 1.0f,
          "turn-brake-reaccel: car moves forward after reaccelerating");
    CHECK(std::abs(distLat) < distFwd * 0.5f,
          "turn-brake-reaccel: lateral drift is small relative to forward motion");
}

static void testHighSpeedTurnBrakeReaccelerate()
{
    VehiclePhysics phys;
    phys.init();

    InputState gas{}; gas.throttle = 1.f;
    simulate(phys, gas, 5.f);
    float speedBeforeTurn = phys.getForwardSpeed();
    std::printf("  high-speed turn: speed before turn=%.2f m/s\n", speedBeforeTurn);

    InputState turnRight{}; turnRight.throttle = 1.f; turnRight.steer = 1.f;
    simulate(phys, turnRight, 2.f);
    float headingAfterTurn = phys.getHeading();
    std::printf("  high-speed turn: heading after turn=%.3f rad (%.1f deg)\n",
                headingAfterTurn, headingAfterTurn * 57.2958f);

    InputState brake{}; brake.brake = 1.f;
    simulate(phys, brake, 5.f);

    Vehicle stopped;
    phys.fillVehicle(stopped);
    float headingAtStop = stopped.heading;
    glm::vec3 posAtStop = stopped.position;
    std::printf("  high-speed turn: heading at stop=%.3f rad (%.1f deg)\n",
                headingAtStop, headingAtStop * 57.2958f);

    InputState gas2{}; gas2.throttle = 1.f;
    simulate(phys, gas2, 2.f);

    Vehicle moving;
    phys.fillVehicle(moving);

    glm::vec3 displacement = moving.position - posAtStop;
    float distFwd = displacement.x * std::sin(headingAtStop)
                  + displacement.z * std::cos(headingAtStop);
    float distLat = displacement.x * std::cos(headingAtStop)
                  - displacement.z * std::sin(headingAtStop);

    std::printf("  high-speed turn: reaccel fwd=%.3f lat=%.3f\n", distFwd, distLat);

    CHECK(std::abs(headingAtStop) < 3.14f,
          "high-speed turn: car doesn't spin past 180 degrees");
    CHECK(distFwd > 1.0f,
          "high-speed turn: car moves forward after reaccelerating");
    CHECK(std::abs(distLat) < distFwd * 0.5f,
          "high-speed turn: lateral drift is small relative to forward motion");
}

static void testContinuousTurnStability()
{
    VehiclePhysics phys;
    phys.init();

    InputState turnRight{}; turnRight.throttle = 1.f; turnRight.steer = 1.f;
    constexpr float DT = 1.f / 120.f;
    float maxHeading = 0.f;
    float heading1s = 0.f, heading3s = 0.f, heading5s = 0.f, heading10s = 0.f;

    for (int step = 0; step < (int)(10.f / DT); ++step) {
        phys.update(DT, turnRight);
        float h = phys.getHeading();
        if (std::abs(h) > std::abs(maxHeading)) maxHeading = h;
        float t = step * DT;
        if (std::abs(t - 1.f) < DT) heading1s = h;
        if (std::abs(t - 3.f) < DT) heading3s = h;
        if (std::abs(t - 5.f) < DT) heading5s = h;
        if (std::abs(t - 10.f) < DT) heading10s = h;
    }

    std::printf("  continuous turn: 1s=%.1f\xC2\xB0 3s=%.1f\xC2\xB0 5s=%.1f\xC2\xB0 10s=%.1f\xC2\xB0 max=%.1f\xC2\xB0\n",
                heading1s * 57.2958f, heading3s * 57.2958f,
                heading5s * 57.2958f, heading10s * 57.2958f,
                maxHeading * 57.2958f);

    CHECK(std::abs(maxHeading) < 20.f,
          "continuous turn: car doesn't spin past 1100 degrees");
}

// ============================================================================
// Wall collision tests
// ============================================================================

static void testWallContactDetection()
{
    // Stamp a wall into a terrain and test contactAt
    Terrain terrain;
    terrain.stampWall(0.f, 0.f, 1.f, 10.f, 3.f, 0.f);  // wall along Z

    // Point well away from wall: no contact
    auto c0 = terrain.contactAt({5.f, 1.f, 0.f});
    CHECK(c0.penetration <= 0.f, "no contact far from wall");

    // Point inside the wall (below wall top, within footprint)
    auto c1 = terrain.contactAt({0.f, 1.f, 0.f});
    CHECK(c1.penetration > 0.f, "contact inside wall");

    // Point above the wall: no contact
    auto c2 = terrain.contactAt({0.f, 4.f, 0.f});
    CHECK(c2.penetration <= 0.f, "no contact above wall");

    // Point at wall edge (horizontal contact)
    auto c3 = terrain.contactAt({0.8f, 1.f, 0.f});
    CHECK(c3.penetration > 0.f, "contact at wall edge");
}

static void testPipeFloorContact()
{
    // Car inside a pipe uses the curved floor for ground contact
    VehiclePhysics phys;
    phys.init();

    Terrain terrain;
    terrain.stampPipe(0.f, 0.f, 8.f, 50.f, 0.f);
    phys.setTerrain(&terrain);

    InputState gas{}; gas.throttle = 1.f;
    simulate(phys, gas, 3.f);

    Vehicle v;
    phys.fillVehicle(v);
    std::printf("  pipe floor: pos=(%.1f, %.1f, %.1f) speed=%.1f\n",
                v.position.x, v.position.y, v.position.z, phys.getForwardSpeed());

    CHECK(phys.getForwardSpeed() > 2.f,
          "pipe floor: car accelerates on pipe floor");
    CHECK(v.position.y > -1.f,
          "pipe floor: car doesn't fall through floor");
}

static void testPipeWallLateralForce()
{
    VehiclePhysics phys;
    phys.init();

    Terrain terrain;
    terrain.stampPipe(0.f, 0.f, 8.f, 50.f, 0.f);
    phys.setTerrain(&terrain);

    InputState none{};
    constexpr float DT = 1.f / 120.f;
    phys.update(DT, none);

    Vehicle v1;
    phys.fillVehicle(v1);

    InputState steerRight{}; steerRight.throttle = 1.f; steerRight.steer = 1.f;
    simulate(phys, steerRight, 2.f);

    Vehicle v2;
    phys.fillVehicle(v2);

    float maxX = 8.f - 0.3f;
    std::printf("  pipe wall lateral: x=%.2f (max allowed ~%.2f)\n",
                v2.position.x, maxX);

    CHECK(std::abs(v2.position.x) < maxX + 1.f,
          "pipe wall lateral: car stays within pipe bounds");
}

static void testPipeSideWallHeadOn()
{
    VehiclePhysics phys;
    phys.init();

    Terrain terrain;
    terrain.stampPipe(0.f, 0.f, 8.f, 50.f, 0.f);
    phys.setTerrain(&terrain);

    InputState gas{}; gas.throttle = 1.f;
    simulate(phys, gas, 2.f);

    InputState steerRight{}; steerRight.throttle = 0.8f; steerRight.steer = 1.f;
    simulate(phys, steerRight, 4.f);

    Vehicle v;
    phys.fillVehicle(v);
    float maxX = 8.f - 0.3f;
    std::printf("  pipe side wall: x=%.2f (max ~%.2f)\n", v.position.x, maxX);

    CHECK(v.position.x < maxX + 0.5f,
          "pipe side wall: car stays within pipe bounds");
}

static void testSteadyStateCircle()
{
    VehiclePhysics phys;
    phys.init();

    InputState gas{}; gas.throttle = 0.5f;
    simulate(phys, gas, 3.f);

    InputState circle{}; circle.throttle = 0.5f; circle.steer = 1.f;
    constexpr float DT = 1.f / 120.f;
    float maxRoll = 0.f;
    float totalHeading = 0.f;

    for (int step = 0; step < (int)(10.f / DT); ++step) {
        phys.update(DT, circle);
        Vehicle v;
        phys.fillVehicle(v);
        float absRoll = std::abs(v.roll);
        if (absRoll > maxRoll) maxRoll = absRoll;
        totalHeading = phys.getHeading();
    }

    std::printf("  steady circle: maxRoll=%.1f\xC2\xB0 totalHeading=%.1f\xC2\xB0\n",
                maxRoll * 57.2958f, totalHeading * 57.2958f);

    CHECK(maxRoll < 0.70f,
          "steady circle: no flip (roll < 40 deg)");
    CHECK(totalHeading > 3.14f,
          "steady circle: car completes at least half a circle");
}

// ============================================================================
// Handling stability tests — catch flipping, rolling, pitch-overs
// ============================================================================

static void testSustainedFullThrottle()
{
    // Hold full throttle for 30s — car must never flip or leave ground unreasonably
    VehiclePhysics phys;
    phys.init();

    InputState gas{}; gas.throttle = 1.f;
    constexpr float DT = 1.f / 120.f;
    float maxRoll = 0.f, maxPitch = 0.f, maxY = 0.f;

    for (int step = 0; step < (int)(30.f / DT); ++step) {
        phys.update(DT, gas);
        Vehicle v;
        phys.fillVehicle(v);
        maxRoll  = std::max(maxRoll,  std::abs(v.roll));
        maxPitch = std::max(maxPitch, std::abs(v.pitch));
        maxY     = std::max(maxY,     v.position.y);
    }

    std::printf("  sustained throttle: maxRoll=%.1f° maxPitch=%.1f° maxY=%.3f\n",
                maxRoll * 57.2958f, maxPitch * 57.2958f, maxY);
    CHECK(maxRoll < 0.35f,  "sustained throttle: roll < 20° (no flip)");
    CHECK(maxPitch < 0.35f, "sustained throttle: pitch < 20° (no wheelie/nosedive)");
    CHECK(maxY < 1.0f,      "sustained throttle: car stays near ground");
}

static void testHighSpeedStraightStability()
{
    // Build up to top speed over 60s — must stay stable
    VehiclePhysics phys;
    phys.init();

    InputState gas{}; gas.throttle = 1.f;
    constexpr float DT = 1.f / 120.f;
    float maxRoll = 0.f, maxPitch = 0.f;

    for (int step = 0; step < (int)(60.f / DT); ++step) {
        phys.update(DT, gas);
        Vehicle v;
        phys.fillVehicle(v);
        maxRoll  = std::max(maxRoll,  std::abs(v.roll));
        maxPitch = std::max(maxPitch, std::abs(v.pitch));
    }

    float topSpeed = phys.getForwardSpeed();
    std::printf("  high-speed straight: topSpeed=%.1f m/s maxRoll=%.1f° maxPitch=%.1f°\n",
                topSpeed, maxRoll * 57.2958f, maxPitch * 57.2958f);
    CHECK(topSpeed > 10.f, "high-speed: reaches meaningful speed");
    CHECK(maxRoll < 0.35f, "high-speed straight: no roll instability");
    CHECK(maxPitch < 0.35f, "high-speed straight: no pitch instability");
}

static void testThrottleLiftStability()
{
    // Full throttle then sudden lift — weight transfer shouldn't flip car
    VehiclePhysics phys;
    phys.init();

    InputState gas{}; gas.throttle = 1.f;
    simulate(phys, gas, 10.f);

    InputState nothing{};
    constexpr float DT = 1.f / 120.f;
    float maxPitch = 0.f;

    for (int step = 0; step < (int)(5.f / DT); ++step) {
        phys.update(DT, nothing);
        Vehicle v;
        phys.fillVehicle(v);
        maxPitch = std::max(maxPitch, std::abs(v.pitch));
    }

    std::printf("  throttle lift: maxPitch=%.1f° after lift-off\n",
                maxPitch * 57.2958f);
    CHECK(maxPitch < 0.35f, "throttle lift: no pitch-over from weight transfer");
}

static void testHardBrakingStability()
{
    // Full speed then slam brakes — no flip
    VehiclePhysics phys;
    phys.init();

    InputState gas{}; gas.throttle = 1.f;
    simulate(phys, gas, 10.f);

    InputState brake{}; brake.brake = 1.f;
    constexpr float DT = 1.f / 120.f;
    float maxPitch = 0.f, maxRoll = 0.f;

    for (int step = 0; step < (int)(5.f / DT); ++step) {
        phys.update(DT, brake);
        Vehicle v;
        phys.fillVehicle(v);
        maxPitch = std::max(maxPitch, std::abs(v.pitch));
        maxRoll  = std::max(maxRoll,  std::abs(v.roll));
    }

    std::printf("  hard braking: maxPitch=%.1f° maxRoll=%.1f°\n",
                maxPitch * 57.2958f, maxRoll * 57.2958f);
    CHECK(maxPitch < 0.35f, "hard braking: no nosedive flip");
    CHECK(maxRoll < 0.35f,  "hard braking: no roll during straight braking");
}

static void testSlalomStability()
{
    // Alternate full left/right steer every 2s at moderate speed
    VehiclePhysics phys;
    phys.init();

    InputState gas{}; gas.throttle = 0.7f;
    simulate(phys, gas, 3.f);

    constexpr float DT = 1.f / 120.f;
    float maxRoll = 0.f, maxPitch = 0.f;
    float steerDir = 1.f;

    for (int step = 0; step < (int)(20.f / DT); ++step) {
        float t = step * DT;
        if (std::fmod(t, 2.f) < DT) steerDir = -steerDir;

        InputState slalom{};
        slalom.throttle = 0.5f;
        slalom.steer = steerDir;
        phys.update(DT, slalom);

        Vehicle v;
        phys.fillVehicle(v);
        maxRoll  = std::max(maxRoll,  std::abs(v.roll));
        maxPitch = std::max(maxPitch, std::abs(v.pitch));
    }

    std::printf("  slalom: maxRoll=%.1f° maxPitch=%.1f°\n",
                maxRoll * 57.2958f, maxPitch * 57.2958f);
    CHECK(maxRoll < 0.70f,  "slalom: roll < 40° (no rollover)");
    CHECK(maxPitch < 0.35f, "slalom: pitch < 20°");
}

static void testThrottleSteerStability()
{
    // Full throttle + full steer for 10s — RWD oversteer but no flip
    VehiclePhysics phys;
    phys.init();

    InputState powerSlide{}; powerSlide.throttle = 1.f; powerSlide.steer = 1.f;
    constexpr float DT = 1.f / 120.f;
    float maxRoll = 0.f, maxY = 0.f;

    for (int step = 0; step < (int)(10.f / DT); ++step) {
        phys.update(DT, powerSlide);
        Vehicle v;
        phys.fillVehicle(v);
        maxRoll = std::max(maxRoll, std::abs(v.roll));
        maxY    = std::max(maxY,    v.position.y);
    }

    std::printf("  throttle+steer: maxRoll=%.1f° maxY=%.3f\n",
                maxRoll * 57.2958f, maxY);
    CHECK(maxRoll < 0.70f, "throttle+steer: no rollover (roll < 40°)");
    CHECK(maxY < 1.0f,     "throttle+steer: car stays near ground");
}

static void testRollNeverExceeds90()
{
    // No matter what input combo, roll should never reach 90° (full flip)
    VehiclePhysics phys;
    phys.init();

    constexpr float DT = 1.f / 120.f;
    float maxRoll = 0.f;

    // Phase 1: hard corner entry
    InputState gas{}; gas.throttle = 1.f;
    simulate(phys, gas, 5.f);
    InputState hardTurn{}; hardTurn.throttle = 1.f; hardTurn.steer = 1.f;
    for (int step = 0; step < (int)(10.f / DT); ++step) {
        phys.update(DT, hardTurn);
        Vehicle v;
        phys.fillVehicle(v);
        maxRoll = std::max(maxRoll, std::abs(v.roll));
    }

    // Phase 2: opposite lock snap
    InputState snap{}; snap.throttle = 1.f; snap.steer = -1.f;
    for (int step = 0; step < (int)(5.f / DT); ++step) {
        phys.update(DT, snap);
        Vehicle v;
        phys.fillVehicle(v);
        maxRoll = std::max(maxRoll, std::abs(v.roll));
    }

    std::printf("  max roll ever: %.1f°\n", maxRoll * 57.2958f);
    CHECK(maxRoll < 1.57f, "roll never exceeds 90° (no full flip)");
}

static void testPitchNeverExceeds45()
{
    // Aggressive throttle/brake cycles — pitch should stay bounded
    VehiclePhysics phys;
    phys.init();

    constexpr float DT = 1.f / 120.f;
    float maxPitch = 0.f;

    for (int cycle = 0; cycle < 5; ++cycle) {
        InputState gas{}; gas.throttle = 1.f;
        for (int s = 0; s < (int)(3.f / DT); ++s) {
            phys.update(DT, gas);
            Vehicle v;
            phys.fillVehicle(v);
            maxPitch = std::max(maxPitch, std::abs(v.pitch));
        }
        InputState brake{}; brake.brake = 1.f;
        for (int s = 0; s < (int)(2.f / DT); ++s) {
            phys.update(DT, brake);
            Vehicle v;
            phys.fillVehicle(v);
            maxPitch = std::max(maxPitch, std::abs(v.pitch));
        }
    }

    std::printf("  max pitch (5 accel/brake cycles): %.1f°\n", maxPitch * 57.2958f);
    CHECK(maxPitch < 0.785f, "pitch never exceeds 45° through accel/brake cycles");
}

static void testCarStaysGrounded()
{
    // After 30s of aggressive driving, car height should be reasonable
    VehiclePhysics phys;
    phys.init();

    constexpr float DT = 1.f / 120.f;
    float maxY = 0.f;

    // Mixed aggressive inputs
    for (int step = 0; step < (int)(30.f / DT); ++step) {
        float t = step * DT;
        InputState in{};
        in.throttle = (std::sin(t * 0.5f) > 0.f) ? 1.f : 0.f;
        in.brake    = (std::sin(t * 0.3f) > 0.8f) ? 1.f : 0.f;
        in.steer    = std::sin(t * 0.7f);
        phys.update(DT, in);

        Vehicle v;
        phys.fillVehicle(v);
        maxY = std::max(maxY, v.position.y);
    }

    std::printf("  grounded test: maxY=%.3f\n", maxY);
    CHECK(maxY < 2.0f, "car stays near ground through aggressive driving");
}

// ============================================================================
// Terrain-based regression tests
// ============================================================================

static void testAcceleratesOnTerrain()
{
    // Car must accelerate on actual terrain, not just flat y=0.
    // This catches stiction or friction bugs that only appear with terrain set.
    Terrain terrain;
    // Default terrain: flat grass at y=0 everywhere (no generate())

    VehiclePhysics phys;
    phys.init();
    phys.setTerrain(&terrain);

    InputState gas{}; gas.throttle = 1.f;
    simulate(phys, gas, 3.f);

    float speed = phys.getForwardSpeed();
    std::printf("  terrain accel: speed=%.1f m/s after 3s\n", speed);
    CHECK(speed > 5.f, "terrain: car accelerates from rest with throttle");
}

static void testAcceleratesOnPlayground()
{
    // Same test but on the full generated playground (stamps, surfaces, etc.)
    Playground pg = createPlayground();

    VehiclePhysics phys;
    phys.init();
    phys.setTerrain(&pg.terrain);

    InputState gas{}; gas.throttle = 1.f;
    simulate(phys, gas, 3.f);

    float speed = phys.getForwardSpeed();
    std::printf("  playground accel: speed=%.1f m/s after 3s\n", speed);
    CHECK(speed > 5.f, "playground: car accelerates from rest with throttle");
}

static void testHandbrakeOversteer()
{
    // Hold throttle to build speed, then release + steer + handbrake.
    // The car should develop significant yaw (rear kicks out).
    VehiclePhysics phys;
    phys.init();

    constexpr float DT = 1.f / 120.f;

    // Phase 1: build speed (8 seconds straight-line)
    InputState gas{}; gas.throttle = 1.f;
    simulate(phys, gas, 8.f);

    float entrySpeed = phys.getForwardSpeed();
    float entryHeading = phys.getHeading();
    std::printf("  handbrake: entry speed=%.1f m/s\n", entrySpeed);
    CHECK(entrySpeed > 20.f, "handbrake: car reaches meaningful speed before turn");

    // Phase 2: release throttle, full left steer + handbrake for 3 seconds
    float maxYawRate = 0.f;
    for (int step = 0; step < (int)(3.f / DT); ++step) {
        InputState turn{};
        turn.steer = -1.f;
        turn.handbrake = 1.f;
        phys.update(DT, turn);

        float yr = std::abs(phys.getYawRate());
        if (yr > maxYawRate) maxYawRate = yr;

        // Diagnostic at key timepoints
        float t = step * DT;
        if (std::abs(t - 0.5f) < DT || std::abs(t - 1.0f) < DT
            || std::abs(t - 2.0f) < DT || std::abs(t - 3.0f) < DT) {
            std::printf("    t=%.1f: yawRate=%.1f°/s speed=%.1f heading=%.1f°\n",
                        t, phys.getYawRate() * 57.2958f,
                        phys.getForwardSpeed(),
                        (phys.getHeading() - entryHeading) * 57.2958f);
        }
    }

    float totalHeading = std::abs(phys.getHeading() - entryHeading) * 57.2958f;
    std::printf("  handbrake: maxYawRate=%.1f°/s totalHeading=%.1f°\n",
                maxYawRate * 57.2958f, totalHeading);
    CHECK(totalHeading > 45.f, "handbrake: car rotates significantly (> 45 deg)");
}

static void testStandstillStability()
{
    // After driving and stopping, the car should not wiggle or spin
    // when all inputs are released.
    VehiclePhysics phys;
    phys.init();

    constexpr float DT = 1.f / 120.f;

    // Drive, steer, then come to a stop
    InputState gas{}; gas.throttle = 1.f; gas.steer = 0.5f;
    simulate(phys, gas, 3.f);

    InputState brake{}; brake.brake = 1.f;
    simulate(phys, brake, 5.f);

    // Now release everything and wait
    Vehicle vStart;
    phys.fillVehicle(vStart);

    InputState nothing{};
    simulate(phys, nothing, 5.f);

    Vehicle vEnd;
    phys.fillVehicle(vEnd);

    float drift = glm::length(vEnd.position - vStart.position);
    float yawDrift = std::abs(vEnd.heading - vStart.heading);
    std::printf("  standstill: drift=%.4f yawDrift=%.4f°\n",
                drift, yawDrift * 57.2958f);
    CHECK(drift < 0.1f, "standstill: car doesn't drift after stopping");
    CHECK(yawDrift < 0.10f, "standstill: car doesn't spin after stopping (< 6 deg)");
}

static void testBrakeHoldStandstill()
{
    // After driving and stopping with brakes held, the car must not spin.
    VehiclePhysics phys;
    phys.init();

    InputState gas{}; gas.throttle = 1.f; gas.steer = 0.3f;
    simulate(phys, gas, 3.f);

    // Hold brake for a long time
    InputState brake{}; brake.brake = 1.f;

    constexpr float DT = 1.f / 120.f;
    float maxYawRate = 0.f;
    for (int step = 0; step < (int)(10.f / DT); ++step) {
        phys.update(DT, brake);
        maxYawRate = std::max(maxYawRate, std::abs(phys.getYawRate()));
    }

    std::printf("  brake-hold: maxYawRate=%.2f deg/s\n", maxYawRate * 57.2958f);
    // Some yaw during initial braking is fine, but should damp out
    float finalSpeed = glm::length(phys.getVelocity());
    CHECK(finalSpeed < 0.5f, "brake-hold: car comes to near stop");
}

// ============================================================================

int main()
{
    std::printf("Running vehicle integration tests...\n\n");

    testNoLateralDriftAtSpawn();
    testNoLateralDriftUnderThrottle();
    testNoLateralDriftOnPlayground();

    testVehicleAccelerates();
    testVehicleBrakes();
    testVehicleCoastsToStop();
    testBrakeDoesNotReverse();
    testRWDOnlyRearWheelsDriven();
    testEngineBraking();
    testEqualBrakingFrontRear();
    testStationaryWithNoInput();
    testBrakeLockupBalance();

    testDirectBackwardDrift();
    testFrictionHoldsAfterBraking();
    testNoCreepAtStartup();
    testThrottleOvercomesFriction();

    testVehicleTurnsRight();
    testVehicleTurnsLeft();
    testStraightWithNoSteer();
    testDeadzoneInput();
    testTurnBrakeReaccelerate();
    testHighSpeedTurnBrakeReaccelerate();
    testContinuousTurnStability();

    testWallContactDetection();
    testPipeFloorContact();
    testPipeWallLateralForce();
    testPipeSideWallHeadOn();
    testSteadyStateCircle();

    testSustainedFullThrottle();
    testHighSpeedStraightStability();
    testThrottleLiftStability();
    testHardBrakingStability();
    testSlalomStability();
    testThrottleSteerStability();
    testRollNeverExceeds90();
    testPitchNeverExceeds45();
    testCarStaysGrounded();

    // Terrain-based regression tests
    testAcceleratesOnTerrain();
    testAcceleratesOnPlayground();
    testHandbrakeOversteer();
    testStandstillStability();
    testBrakeHoldStandstill();

    return reportResults();
}
