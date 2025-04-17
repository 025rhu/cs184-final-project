#pragma once
#include <string>
#include <Eigen/Core>
#include <nanogui/opengl.h>          

struct MeshFrame {
    GLuint vao        = 0;    // the VAO we'll bind when drawing
    GLuint indexCount = 0;    // number of indices to pass to glDrawElements
};

/// Load a *static* mesh (one FBX file = one frame).
/// Returns a fully‑populated MeshFrame. If loading fails, vao == 0.
MeshFrame loadModel(const std::string&  path,
                    Eigen::Vector3f*    outCenter  = nullptr,
                    float*              outHeight  = nullptr);
