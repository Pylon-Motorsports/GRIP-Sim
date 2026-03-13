#pragma once
#include "Tire.hpp"

class SimpleTire : public Tire {
public:
    // Static (peak) and dynamic (sliding) friction coefficients
    // BRZ on 200-treadwear sport tires: ~1.2 mu in all directions.
    // Symmetric grip circle — real tires have roughly equal lat/lon grip.
    float lateralMuStatic  = 1.2f;   // peak lateral mu (before breakaway)
    float lateralMuDynamic = 0.95f;  // sliding lateral mu
    float longMuStatic     = 1.2f;   // peak longitudinal mu (matched to lateral)
    float longMuDynamic    = 0.95f;  // sliding longitudinal mu

    SimpleTire(std::string name, glm::vec3 attachPt, bool steered_, bool driven_);

protected:
    ComponentOutput compute(const ComponentInput& input) override;
};
