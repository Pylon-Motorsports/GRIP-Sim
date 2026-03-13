#pragma once
#include "SuspensionCorner.hpp"

class Subframe : public VehiclePhysicsComponent {
public:
    float halfTrack    = 0.76f;    // half track width (m)
    float rideHeight   = 0.30f;    // mount height above ground (m)
    float tireDropM    = 0.30f;    // tire center below mount (m)
    bool  steered      = false;
    bool  driven       = false;

    Subframe(std::string name, glm::vec3 attachPt, bool steered_, bool driven_);

    void init() override;

    SuspensionCorner* left();
    SuspensionCorner* right();
};
