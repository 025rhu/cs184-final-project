// AnimatedObject.h
#pragma once
#include <vector>
#include <string>
#include <nanogui/opengl.h>	
#include "model.h"

class AnimatedObject {
public:
    /// load a bunch of frames
    bool load(const std::vector<std::string>& paths);

    // advance the time, tracked by accumulator
    void update(float dt);                 // dt = seconds since previous call

    // draw the current frame
    void draw() const;

    // default 1fps
    float secondsPerFrame = 1.0f;          // 1 fps

private:
    std::vector<MeshFrame> frames;
    int  current = 0;
    float accumulator = 0.0f;
};
