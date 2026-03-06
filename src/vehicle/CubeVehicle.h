#pragma once
#include "IVehicleDynamics.h"
#include "road/TerrainQuery.h"
#include "rendering/RoadMesh.h"
#include <glm/glm.hpp>
#include <vector>

/// Bare-metal vehicle: a body and 4 wheels. No physics whatsoever.
/// Inputs are not connected to any output. Build up from here.
class CubeVehicle : public IVehicleDynamics {
public:
    struct Params {
        float massKg       { 1200.f };
        float halfTrackM   { 0.75f };   ///< Half-track width (CG to wheel, X)
        float frontAxleM   { 1.3f };    ///< CG to front axle (Z)
        float rearAxleM    { 1.4f };    ///< CG to rear axle (Z)
        float cgHeightM    { 0.45f };   ///< CG height above ground (Y)
        float wheelRadiusM { 0.32f };   ///< Wheel radius
    };

    explicit CubeVehicle(Params params = {});

    void reset(glm::vec3 position, float headingRad) override;
    void integrate(const InputFrame& input, float dt) override;
    [[nodiscard]] const VehicleState& state() const override { return state_; }

    void setTerrainQuery(const TerrainQuery& terrain);
    void setTrees(const std::vector<TreeInstance>& trees);
    void setCenterlinePoints(const std::vector<glm::vec3>& points,
                             const std::vector<uint32_t>&  segmentStartVertex,
                             int verticesPerRow);

private:
    Params params_;
    VehicleState state_;
    TerrainQuery terrain_;
    std::vector<TreeInstance> trees_;

    glm::vec3 pos_  { 0.f };
    float     yaw_  { 0.f };

    // Segment tracking
    std::vector<glm::vec3> centerline_;
    std::vector<uint32_t>  segmentStartVertex_;
    int                    vertsPerRow_ { 7 };

    void updateWheelPositions();
    void updateSegmentTracking();
};
