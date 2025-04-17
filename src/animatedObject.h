// AnimatedObject.h
#pragma once
#include <vector>
#include <string>
#include <nanogui/opengl.h>	
#include "model.h"

class AnimatedObject {
public:
    /// Load a sequence of static FBX files, one per frame.
    bool load(const std::vector<std::string>& paths);

    /// Advance the internal time and pick the frame to display.
    void update(float dt);                 // dt = seconds since previous call

    /// Draw the current frame.
    void draw() const;

    /// Seconds per frame (you can change this on the fly).
    float secondsPerFrame = 1.0f;          // = 10 fps

private:
    std::vector<MeshFrame> frames;
    int  current      = 0;
    float accumulator = 0.0f;
};
