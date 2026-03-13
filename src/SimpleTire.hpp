#pragma once
#include "Tire.hpp"

class SimpleTire : public Tire {
public:
    // Static (peak) and dynamic (sliding) friction coefficients
    float lateralMuStatic  = 1.2f;   // peak lateral mu (before breakaway)
    float lateralMuDynamic = 0.95f;  // sliding lateral mu
    float longMuStatic     = 1.1f;   // peak longitudinal mu
    float longMuDynamic    = 0.85f;  // sliding longitudinal mu

    SimpleTire(std::string name, glm::vec3 attachPt, bool steered_, bool driven_);

protected:
    ComponentOutput compute(const ComponentInput& input) override;
};
