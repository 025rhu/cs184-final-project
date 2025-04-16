// #ifndef NODE_H
// #define NODE_H
// #include <vector>
// #include "CGL/CGL.h"
// #include "CGL/quaternion.h"
// #include "CGL/vector3D.h"
// // #include "Eigen/Core"

// struct VertexAttributes {
//     CGL::Vector3D position;
//     CGL::Vector3D normal;
//     CGL::Vector3D color;  // RGB values (0–1)

//     VertexAttributes interpolate(const VertexAttributes& other, float alpha) const {
//         VertexAttributes result;
//         result.position = (1 - alpha) * position + alpha * other.position;
//         result.normal   = (1 - alpha) * normal + alpha * other.normal;
//         result.color    = (1 - alpha) * color + alpha * other.color;
//         return result;
//     }
// };


// struct KeyframeState {
//     float time;  // in seconds

//     // Transform
//     CGL::Vector3D position;
//     CGL::Quaternion rotation;
//     CGL::Vector3D scale;

//     std::vector<VertexAttributes> vertex_data;
// };


// class AnimatedObject {
//     public:
//         std::string name;
//         std::vector<VertexAttributes> baseVertices;
//         std::vector<KeyframeState> keyframes;
    
//         KeyframeState interpolateAt(float time) const;
// };
    
    

// #endif // NODE_H
