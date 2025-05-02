#ifndef ANIMATOR_H
#define ANIMATOR_H

// #include "Eigen/src/Geometry/Quaternion.h"
#include "nanogui/common.h"
#include <glad/glad.h>
#include <nanogui/screen.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include <unordered_map>
#include <assimp/scene.h>
// #include "shader.h"

using namespace nanogui;
using namespace std;


struct Vertex {
    Vector3f position;
    Vector3f normal;
    Vector3f color;
    GLuint boneIndices[4];
    float  weights[4];  
    // vector<GLuint> boneIndices; changed bc apparently this is bad for malloc
    // vector<float> weights;  
};


struct Bone {
    Bone();
    ~Bone();

    int id;

// #################### ATTRIBUTES ####################
    // identifier for this bone (aiBone.mName)
    string name;
    // transforms from mesh space to bone space in bind pose (aiBone.mOffsetMatrix)
    Matrix4f offsetMatrix;
    // list of points to all children of this bone in the overall bone hierarchy (aiNode.mChildren)
    vector<Bone*> children;
    // transformation relative to the node's parent at rest (aiNode.mTransformation)
    Matrix4f restLocalTransformation;
    // Vector3f restPosition;
    // Eigen::Quaternionf restRotation;
    // Vector3f restScaling;

    // transformation relative to the node's parent during the animation (interpolated)
    Matrix4f localTransformation;
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
    void interpolateAt(double time, Matrix4f &parentTransform);

private:
    Vector3f interpolatePosition(double time) const;
    Eigen::Quaternionf interpolateRotation(double time) const;
    Vector3f interpolateScaling(double time) const;

    int findIndex(double time, const vector<double>* times) const;

    // int findPositionIndex(double time) const;
    // int findRotationIndex(double time) const;
    // int findScalingIndex(double time) const;

    Matrix4f buildLocalTransform(double time);

};


struct Mesh {
    // mesh architecture
    Mesh();
    ~Mesh();
    std::vector<Eigen::Matrix4f> boneMatrices;
    unordered_map<string, Bone*>* bones;
    Bone* rootBone;
    // TENTATIVELY REMOVED. ADD BACK IF NEEDED LATER.
    // double duration = 0.0;
    // double ticksPerSecond = 25.0;
    // vector<Vertex>* vertices;


    void animateAt(double time);
    // vector<Matrix4f>* getBoneMatrices();
    // helps with loading the model from FBX file into proper values
    void retrieveSceneValues(const aiScene* scene);

    // interpolate and populate bone matrices
    // void findFinalBoneMatrices(double time, vector<Eigen::Matrix4f>& boneMatrices);
    
    // put bone matrices in bone matrix array after interpolation
    // void getBoneMatrices(Bone* bone, vector<Eigen::Matrix4f>& boneMatrices);
    // void getBoneMatrices();

    // Vector3f bboxMin;
    // Vector3f bboxMax;

    void debugBones();
private:

    // helper function with loading the model from FBX file into proper values
    // void buildBoneHierarchy(const aiNode* node, Bone* parent);

};


class Animation {
public:
Animation(const std::string &fbxPath, const GLuint shader);
~Animation();

    // load a model from the FBX file
    // void loadModel(const std::string& path);

    // Advance the skeleton to the given time (in seconds)
    void animateAt(double time);

    // draw the skinned mesh (upload bone matrices to GPU) after interpolation
    void draw();

    Mesh* character;      // your mesh + bone hierarchy
    double startTime = -1;

private:
    // skin‐shader program
    GLuint skinProgram;         // shader ID
    GLint locBoneMatrices;      // uniform location for uBoneMatrices[]
    GLuint locModel_;           // uniform location for model
    // VAO + how many indices to draw
    GLuint VAO;
    GLsizei indexCount;

    // Helpers
    void initMeshBuffers(const aiScene* scene);
};


#endif // ANIMATOR_H