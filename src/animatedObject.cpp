// AnimatedObject.cpp
#include "animatedObject.h"
#include "model.h"          // ← your loadModel‑like helper
#include <iostream>

/// helper that **returns** a VAO + index count instead of writing globals
static MeshFrame loadStaticMesh(const std::string& path) {
    MeshFrame f = loadModel(path);
    if (f.vao == 0) {                 // 0 VAO = failure
        std::cerr << "Cannot load " << path << '\n';
    }
    return f;
}

bool AnimatedObject::load(const std::vector<std::string>& paths)
{
    frames.clear();
    for (const std::string& p : paths) {
        MeshFrame frame = loadStaticMesh(p);
        if (frame.vao == 0) {
            // bad keyframe
            return false;
        }
        frames.push_back(frame);
    }
    return !frames.empty();
}

void AnimatedObject::update(float dt)
{
    if (frames.size() < 2) return;

    accumulator += dt;
    if (accumulator >= secondsPerFrame) {
        accumulator -= secondsPerFrame;

        // cycle
        current = (current + 1) % int(frames.size());
    }
}

void AnimatedObject::draw() const
{
    if (frames.empty()) {
        return;
    }
    const MeshFrame& f = frames[current];
    glBindVertexArray(f.vao);
    glDrawElements(GL_TRIANGLES, f.indexCount, GL_UNSIGNED_INT, nullptr);
    glBindVertexArray(0);
}
