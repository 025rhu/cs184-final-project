#ifndef ANIMATOR_H
#define ANIMATOR_H

// #include "Eigen/src/Geometry/Quaternion.h"
#include "nanogui/common.h"
#include <glad/glad.h>
#include <nanogui/screen.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>

using namespace nanogui;
using namespace std;


struct Vertex {
    Vector3f position;
    Vector3f normal;
    Vector3f color;
    vector<GLuint> boneIndices;
    vector<float> weights;
    
};


struct Bone {
    Bone() {};
    ~Bone();

// #################### ATTRIBUTES ####################
    // identifier for this bone (aiBone.mName)
    string name;
    // transforms from mesh space to bone space in bind pose (aiBone.mOffsetMatrix)
    Matrix4f offsetMatrix;
    // list of points to all children of this bone in the overall bone hierarchy (aiNode.mChildren)
    vector<Bone*>* children;
    // transformation relative to the node's parent at rest (aiNode.mTransformation)
    Matrix4f restLocalTransformation;
    // Vector3f restPosition;
    // Eigen::Quaternionf restRotation;
    // Vector3f restScaling;

    // transformationn relative to the node's parent during the animation (interpolated)
    // Matrix4f localTransformation;
    // total transformation of this bone at some time during the animation (interpolated, local * parent)
    Matrix4f globalTransformation;

    // times corresponding to each position key (aiNodeAnim.mPositionKeys[i].mTime)
    std::vector<double> positionTimes;
    // position keys of this animation channel (aiNodeAnim.mPositionKeys)
    vector<Eigen::Vector3f> positionKeys;

    // times corresponding to each rotation key (aiNodeAnim.mRotationKeys[i].mTime)
    vector<double> rotationTimes;
    // rotation keys of this animation channel (aiNodeAnim.mRotationKeys)
    vector<Eigen::Quaternionf> rotationKeys;
    
    // times corresponding to each scaling key (aiNodeAnim.mScalingKeys[i].mTime)
    vector<double> scalingTimes;
    // scaling keys of this animation channel (aiNodeAnim.mScalingKeys)
    vector<Eigen::Vector3f> scalingKeys;
    // vector<Vertex> vertices;

// #################### FUNCTIONS ####################
    void interpolateAt(double time, Matrix4f parentTransform);

private:
    Vector3f interpolatePosition(double time) const;
    Eigen::Quaternionf interpolateRotation(double time) const;
    Vector3f interpolateScaling(double time) const;

    int findPositionIndex(double time) const;
    int findRotationIndex(double time) const;
    int findScalingIndex(double time) const;

    Matrix4f buildLocalTransform(double time);
};


struct Mesh {
    // variables for rendering purposes
    GLuint VAO=0;
    GLuint indexCount;
    // mesh architecture
    unordered_map<string, Bone*>* bones;
    Bone* rootBone;
    vector<Vertex>* vertices;

    void animateAt(double time);
};


struct Animation {
    Mesh* character; // full object
    double duration;

    double startTime;
    double endTime;
    double timestep;

    void animate();
};


#endif // ANIMATOR_H