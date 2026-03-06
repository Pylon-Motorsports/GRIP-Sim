#pragma once
#include "VehicleMesh.h"
#include "vehicle/VehicleState.h"
#include <vector>
#include <string>
#include <cstdint>

/// Generates HUD text geometry using a 5×7 bitmap font.
/// Renders as colored quads using the existing VehicleVertex format.
/// Call buildFrame() each frame with the current vehicle state,
/// then draw the resulting vertex/index buffers in screen space
/// with an orthographic projection.
class HudOverlay {
public:
    /// Rebuild all HUD geometry for the current frame.
    void buildFrame(const VehicleState& state, float windowW, float windowH);

    const std::vector<VehicleVertex>& vertices() const { return verts_; }
    const std::vector<uint32_t>&      indices()  const { return idxs_;  }
    bool empty() const { return idxs_.empty(); }

private:
    std::vector<VehicleVertex> verts_;
    std::vector<uint32_t>      idxs_;

    /// Draw a single character at (x,y) with given pixel size and color.
    void drawChar(char ch, float x, float y, float pixSize,
                  float r, float g, float b, float a = 1.f);

    /// Draw a string starting at (x,y). Returns width of rendered string.
    float drawString(const std::string& text, float x, float y, float pixSize,
                     float r, float g, float b, float a = 1.f);

    /// Draw a filled rectangle.
    void drawRect(float x, float y, float w, float h,
                  float r, float g, float b, float a = 1.f);

    /// Add a labeled value line. Returns the Y offset consumed.
    float drawLabelValue(const std::string& label, const std::string& value,
                         float x, float y, float pixSize,
                         float lr, float lg, float lb,
                         float vr, float vg, float vb);

    /// 5×7 bitmap font lookup. Returns 7 bytes (rows), each 5 bits wide.
    static const uint8_t* glyphData(char ch);
};
