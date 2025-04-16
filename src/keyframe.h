// #ifndef KEYFRAME_H
// #define KEYFRAME_H

// #include <string>
// #include <unordered_map>
// #include <vector>
// #include "CGL/vector3D.h"
// #include "CGL/matrix4x4.h"

// // Holds mesh data for an object at a keyframe
// struct GeometrySnapshot {
//   std::vector<CGL::Vector3D> positions;
//   std::vector<CGL::Vector3D> normals;
//   std::vector<CGL::Vector3D> colors;
//   std::vector<unsigned int> indices;
// };

// // Transform + geometry for a single object at a keyframe
// struct ObjectState {
//   std::string name;
//   CGL::Matrix4x4 transform;
//   GeometrySnapshot geometry;
// };

// class KeyFrame {
// public:
//   KeyFrame(int timestamp);

//   int timestamp;
//   std::unordered_map<std::string, ObjectState> objects;

//   void addObject(const ObjectState& object_state);
//   bool hasObject(const std::string& name) const;

//   const ObjectState* getObject(const std::string& name) const;
//   ObjectState* getObject(const std::string& name);
// };

// #endif // KEYFRAME_H
