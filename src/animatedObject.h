#pragma once
#include <vector>
#include <string>
#include <nanogui/opengl.h>	
#include "model.h"

class AnimatedObject {
public:
    // load frames
    bool load(const std::vector<std::string>& paths);

    // advance the time, tracked by accumulator
    void update(float dt);

    // draw frame
    void draw() const;

    float secondsPerFrame = 1.0f;

private:
    std::vector<MeshFrame> frames;
    int  current = 0;
    float accumulator = 0.0f;
};
