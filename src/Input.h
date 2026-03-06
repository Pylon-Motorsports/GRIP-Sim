#pragma once

struct InputState {
    float throttle = 0.f;  // [0, 1]
    float brake    = 0.f;  // [0, 1]
    float steer    = 0.f;  // [-1, 1] negative=left, positive=right
};
