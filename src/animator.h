#ifndef ANIMATOR_H
#define ANIMATOR_H

#include <glad/glad.h>
#include <nanogui/screen.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>

using namespace nanogui;
using namespace std;


class Vertex {
public:
    Vector3f position;
    Vector3f normal;
    Vector3f color;
    std::vector<GLuint> boneIndices;
    std::vector<float> weights;
    
};


class Bone {
public:
    string name;
    Matrix4f offsetMatrix;
    std::vector<Bone*>* children;
    Matrix4f transformation;

    vector<double> positionTimes;
    vector<Eigen::Vector3f> positionKeys;

    vector<double> rotationTimes;
    vector<Eigen::Quaternionf> rotationKeys;
    
    vector<double> scalingTimes;
    vector<Eigen::Vector3f> scalingKeys;
    // std::vector<Vertex> vertices;
};


class Mesh {
    GLuint VAO=0;
    GLuint indexCount;
    std::unordered_map<string, Bone*>* bones;
    vector<Vertex>* vertices;
};


class Animation {
public:
    Mesh* character; // full object
    double duration;
    
};


#endif // ANIMATOR_H