#include "CubeVehicle.h"
#include <cmath>

CubeVehicle::CubeVehicle(Params params)
    : params_(params)
{
    reset({}, 0.f);
}

void CubeVehicle::reset(glm::vec3 position, float headingRad)
{
    pos_ = position;
    yaw_ = headingRad;

    // Place CG at correct height above ground
    float groundY = terrain_.empty() ? 0.f
        : terrain_.heightAt(position.x, position.z, position.y);
    if (groundY <= TerrainQuery::NO_GROUND + 1.f) groundY = 0.f;
    pos_.y = groundY + params_.cgHeightM;

    state_ = VehicleState{};
    state_.position   = pos_;
    state_.headingRad = yaw_;

    updateWheelPositions();
}

void CubeVehicle::integrate(const InputFrame& /*input*/, float /*dt*/)
{
    // No physics. Inputs are not connected to anything.
    // State remains exactly as set by reset().

    state_.position   = pos_;
    state_.headingRad = yaw_;

    updateWheelPositions();
    updateSegmentTracking();
}

void CubeVehicle::updateWheelPositions()
{
    glm::vec3 fwd   { std::sin(yaw_), 0.f, std::cos(yaw_) };
    glm::vec3 right { std::cos(yaw_), 0.f, -std::sin(yaw_) };

    float hw = params_.halfTrackM;
    float lf = params_.frontAxleM;
    float lr = params_.rearAxleM;
    float rw = params_.wheelRadiusM;

    // Wheel hub positions: FL, FR, RL, RR
    glm::vec3 hubs[4] = {
        pos_ + fwd * lf - right * hw,  // FL
        pos_ + fwd * lf + right * hw,  // FR
        pos_ - fwd * lr - right * hw,  // RL
        pos_ - fwd * lr + right * hw,  // RR
    };

    for (int c = 0; c < 4; ++c) {
        float groundY = terrain_.empty() ? 0.f
            : terrain_.heightAt(hubs[c].x, hubs[c].z, pos_.y);
        if (groundY <= TerrainQuery::NO_GROUND + 1.f) groundY = 0.f;

        hubs[c].y = groundY + rw;
        state_.wheelPos[c] = hubs[c];
        state_.wheelGroundHeight[c] = groundY;
        state_.suspTopPos[c] = hubs[c];
        state_.suspTopPos[c].y = pos_.y;
        state_.suspLength[c] = pos_.y - (groundY + rw);
    }
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
