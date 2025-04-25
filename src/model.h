#pragma once
#include <string>
#include <Eigen/Core>
#include <nanogui/opengl.h>          


/*
- Vertex Array Object (VAO): an object that contains one or more Vertex Buffer Objects (VBO) and is designed to store the information for a complete rendered object
- vertex buffer object (VBO): an OpenGL feature that provides methods for uploading vertex data to the video device for non-immediate-mode rendering
- glDrawElement: a function that draws primitives (like triangles, lines, or points) from pre-existing vertex and index data stored in memory buffers
*/


/*
represents one mesh.
- vao: 
*/
struct MeshFrame {
    GLuint vao        = 0;    // the VAO we'll bind when drawing
    GLuint indexCount = 0;    // number of indices to pass to glDrawElements
};

/// Load a *static* mesh (one FBX file = one frame).
/// Returns a fully‑populated MeshFrame. If loading fails, vao == 0.
MeshFrame loadModel(const std::string&  path,
                    Eigen::Vector3f*    outCenter  = nullptr,
                    float*              outHeight  = nullptr);
