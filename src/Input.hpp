#pragma once

struct InputState {
    float throttle  = 0.f;  // [0, 1]
    float brake     = 0.f;  // [0, 1]
    float steer     = 0.f;  // [-1, 1] negative=left, positive=right
    float handbrake = 0.f;  // [0, 1] rear-only brake
    bool  clutchIn  = false; // true = clutch disengaged (engine free-revs)
};
