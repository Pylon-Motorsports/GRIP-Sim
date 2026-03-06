#include "CubeVehicle.h"
#include <cmath>
#include <algorithm>
#include <glm/gtc/constants.hpp>

static constexpr float G   = 9.81f;
static constexpr float EPS = 0.01f;

CubeVehicle::CubeVehicle(Params params)
    : params_(params)
{
    reset({}, 0.f);
}

void CubeVehicle::reset(glm::vec3 position, float headingRad)
{
    pos_ = position;
    yaw_ = headingRad;

    float groundY = terrain_.empty() ? 0.f : terrain_.heightAt(position.x, position.z, position.y);
    if (groundY <= TerrainQuery::NO_GROUND + 1.f) groundY = 0.f;
    pos_.y = groundY + params_.halfHeight;

    vel_     = {};
    yawRate_ = 0.f;

    state_ = VehicleState{};
    state_.position   = pos_;
    state_.headingRad = yaw_;
    state_.currentGear = 1;
    state_.engineRpm   = 800.f;
}

void CubeVehicle::integrate(const InputFrame& input, float dt)
{
    const float m = params_.massKg;

    // Forward/right directions from heading
    glm::vec3 fwd   { std::sin(yaw_), 0.f, std::cos(yaw_) };
    glm::vec3 right { std::cos(yaw_), 0.f, -std::sin(yaw_) };

    // -----------------------------------------------------------------------
    // 1. Ground contact — query terrain at cube centre
    // -----------------------------------------------------------------------
    float groundY = terrain_.empty() ? 0.f : terrain_.heightAt(pos_.x, pos_.z, pos_.y);
    bool onGround = (groundY > TerrainQuery::NO_GROUND + 1.f);
    if (!onGround) groundY = -1000.f;  // free-fall

    float floorY = groundY + params_.halfHeight;

    // -----------------------------------------------------------------------
    // 2. Forces
    // -----------------------------------------------------------------------
    glm::vec3 totalForce { 0.f, -m * G, 0.f };  // gravity

    if (onGround) {
        // Ground normal force (spring-damper contact)
        if (pos_.y < floorY) {
            float penetration = floorY - pos_.y;
            float springForce = penetration * 50000.f;
            float dampForce   = -std::min(vel_.y, 0.f) * 8000.f;
            totalForce.y += springForce + dampForce;
        }

        // Throttle → forward force
        float driveForce = input.throttle * params_.maxForceN;
        totalForce += fwd * driveForce;

        // Brake → opposing force
        float speed = glm::length(vel_);
        if (speed > EPS && input.brake > 0.f) {
            float brakeForce = input.brake * params_.maxBrakeN;
            brakeForce = std::min(brakeForce, speed * m / dt);  // don't reverse
            totalForce -= (vel_ / speed) * brakeForce;
        }

        // Friction — opposes lateral sliding
        float vLat = glm::dot(vel_, right);
        float lateralFriction = params_.frictionMu * m * G;
        float latForce = std::min(std::abs(vLat) * m / dt, lateralFriction);
        if (std::abs(vLat) > EPS)
            totalForce -= right * std::copysign(latForce, vLat);

        // Rolling/velocity drag
        if (speed > EPS) {
            float dragForce = params_.dragCoeff * speed * speed + 0.01f * m * G;
            totalForce -= (vel_ / speed) * dragForce;
        }

        // Steering → yaw rate
        yawRate_ = input.steer * params_.maxSteerRad * std::min(1.f, speed / 5.f);
    }

    // Slope force — gravity component along slope
    if (onGround) {
        float halfWB = params_.halfLength;
        float hFront = terrain_.heightAt(pos_.x + fwd.x * halfWB, pos_.z + fwd.z * halfWB, groundY);
        float hRear  = terrain_.heightAt(pos_.x - fwd.x * halfWB, pos_.z - fwd.z * halfWB, groundY);
        if (hFront > TerrainQuery::NO_GROUND + 1.f && hRear > TerrainQuery::NO_GROUND + 1.f) {
            float sinSlope = (hFront - hRear) / (2.f * halfWB);
            totalForce -= fwd * (m * G * sinSlope);
        }
    }

    // -----------------------------------------------------------------------
    // 3. Integration
    // -----------------------------------------------------------------------
    glm::vec3 accel = totalForce / m;
    vel_ += accel * dt;
    pos_ += vel_  * dt;

    // Hard ground clamp
    if (onGround && pos_.y < floorY) {
        pos_.y = floorY;
        if (vel_.y < 0.f) vel_.y = 0.f;
    }

    // Yaw integration
    yaw_ += yawRate_ * dt;

    // -----------------------------------------------------------------------
    // 4. Tree collision
    // -----------------------------------------------------------------------
    if (!trees_.empty()) {
        float halfDiag = std::sqrt(params_.halfLength * params_.halfLength
                                 + params_.halfWidth * params_.halfWidth);

        for (const auto& tree : trees_) {
            float dx = pos_.x - tree.position.x;
            float dz = pos_.z - tree.position.z;
            float dist2 = dx * dx + dz * dz;
            float minDist = halfDiag + tree.trunkRadius;
            if (dist2 < minDist * minDist && dist2 > 0.0001f) {
                float dist = std::sqrt(dist2);
                float penetration = minDist - dist;
                float nx = dx / dist;
                float nz = dz / dist;

                pos_.x += nx * penetration;
                pos_.z += nz * penetration;

                float vInto = vel_.x * nx + vel_.z * nz;
                if (vInto < 0.f) {
                    vel_.x -= nx * vInto * 1.1f;
                    vel_.z -= nz * vInto * 1.1f;
                    float impactSpeed = -vInto;
                    if (impactSpeed > 1.f) {
                        float energyLoss = std::min(0.5f, impactSpeed * 0.05f);
                        vel_ *= (1.f - energyLoss);
                        float crossProduct = nx * std::sin(yaw_) + nz * std::cos(yaw_);
                        yawRate_ += crossProduct * impactSpeed * 0.3f;
                    }
                }
            }
        }
    }

    // NaN guard
    if (std::isnan(pos_.x) || std::isnan(pos_.y) || std::isnan(pos_.z)) {
        vel_ = {};
        yawRate_ = 0.f;
    }

    // -----------------------------------------------------------------------
    // 5. Update VehicleState
    // -----------------------------------------------------------------------
    fwd   = { std::sin(yaw_), 0.f, std::cos(yaw_) };
    right = { std::cos(yaw_), 0.f, -std::sin(yaw_) };
    float vFwd = glm::dot(vel_, fwd);

    state_.position    = pos_;
    state_.velocity    = vel_;
    state_.headingRad  = yaw_;
    state_.speedMs     = vFwd;
    state_.throttle    = input.throttle;
    state_.brake       = input.brake;
    state_.steer       = input.steer;
    state_.rollRad     = 0.f;
    state_.pitchRad    = 0.f;
    state_.weightFront = 0.5f;
    state_.odoMeters  += std::max(0.f, vFwd) * dt;

    // Fill wheel positions at cube corners (for renderer compatibility)
    float hw = params_.halfWidth;
    float hl = params_.halfLength;
    glm::vec3 corners[4] = {
        pos_ + fwd * hl - right * hw,  // FL
        pos_ + fwd * hl + right * hw,  // FR
        pos_ - fwd * hl - right * hw,  // RL
        pos_ - fwd * hl + right * hw,  // RR
    };
    for (int c = 0; c < 4; ++c) {
        corners[c].y = onGround ? groundY : pos_.y - params_.halfHeight;
        state_.wheelPos[c] = corners[c];
        state_.wheelGroundHeight[c] = onGround ? groundY : TerrainQuery::NO_GROUND;
        state_.suspTopPos[c] = corners[c];
        state_.suspTopPos[c].y = pos_.y + params_.halfHeight * 0.5f;
        state_.suspLength[c] = params_.halfHeight;
    }

    updateSegmentTracking();
}

void CubeVehicle::setTerrainQuery(const TerrainQuery& terrain)
{
    terrain_ = terrain;
}

void CubeVehicle::setTrees(const std::vector<TreeInstance>& trees)
{
    trees_ = trees;
}

void CubeVehicle::setCenterlinePoints(
    const std::vector<glm::vec3>& points,
    const std::vector<uint32_t>&  segmentStartVertex,
    int vertsPerRow)
{
    centerline_         = points;
    segmentStartVertex_ = segmentStartVertex;
    vertsPerRow_        = vertsPerRow;
}

void CubeVehicle::updateSegmentTracking()
{
    if (centerline_.empty()) return;

    float minDist = 1e9f;
    int   bestIdx = 0;
    for (int i = 0; i < (int)centerline_.size(); ++i) {
        glm::vec3 d = state_.position - centerline_[i];
        d.y = 0.f;
        float dist = glm::dot(d, d);
        if (dist < minDist) { minDist = dist; bestIdx = i; }
    }

    int seg = 0;
    for (int s = (int)segmentStartVertex_.size() - 1; s >= 0; --s) {
        int firstPoint = segmentStartVertex_[s] / vertsPerRow_;
        if (bestIdx >= firstPoint) { seg = s; break; }
    }

    state_.segmentIndex    = seg;
    state_.segmentProgress = 0.f;
}
