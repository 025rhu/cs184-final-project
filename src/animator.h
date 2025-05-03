#ifndef ANIMATOR_H
#define ANIMATOR_H

#include "nanogui/common.h"
#include <glad/glad.h>
#include <nanogui/screen.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include <unordered_map>
#include <assimp/scene.h>

using namespace nanogui;
using namespace std;


struct Vertex {
    Vector3f position;
    Vector3f normal;
    Vector3f color;
    GLuint boneIndices[4];
    float  weights[4];
};


struct Bone {
    Bone();
    ~Bone();

    int id;

// #################### ATTRIBUTES ####################
    // identifier for this bone (aiBone.mName)
    string name;
    Matrix4f offsetMatrix;      // transforms from mesh space to bone space in bind pose (aiBone.mOffsetMatrix)

    vector<Bone*> children;     // list of points to all children of this bone in the overall bone hierarchy (aiNode.mChildren)

    Matrix4f restLocalTransformation;   // transformation relative to the node's parent at rest (aiNode.mTransformation)
    Matrix4f localTransformation;       // transformation relative to the node's parent during the animation (interpolated)
    Matrix4f globalTransformation;      // total transformation of this bone at some time during the animation (interpolated, local * parent)

    
    std::vector<double> positionTimes;      // times corresponding to each position key (aiNodeAnim.mPositionKeys[i].mTime)
    vector<Eigen::Vector3f> positionKeys;   // position keys of this animation channel (aiNodeAnim.mPositionKeys)

    vector<double> rotationTimes;           // times corresponding to each rotation key (aiNodeAnim.mRotationKeys[i].mTime)
    vector<Eigen::Quaternionf> rotationKeys;    // rotation keys of this animation channel (aiNodeAnim.mRotationKeys)
    
    vector<double> scalingTimes;            // times corresponding to each scaling key (aiNodeAnim.mScalingKeys[i].mTime)
    vector<Eigen::Vector3f> scalingKeys;    // scaling keys of this animation channel (aiNodeAnim.mScalingKeys)

// #################### FUNCTIONS ####################
    void interpolateAt(double time, Matrix4f &parentTransform);
    void applyRestPose(const Matrix4f &parentTransform);

private:
    Vector3f interpolatePosition(double time) const;
    Eigen::Quaternionf interpolateRotation(double time) const;
    Vector3f interpolateScaling(double time) const;

    int findIndex(double time, const vector<double>* times) const;
    Matrix4f buildLocalTransform(double time);

};


struct Mesh {
    Mesh();
    ~Mesh();
    std::vector<Eigen::Matrix4f> boneMatrices;
    unordered_map<string, Bone*>* bones;
    Bone* rootBone;
    double duration = 0.0;
    double ticksPerSecond;

    void animateAt(double time);
    void retrieveSceneValues(const aiScene* scene); // helps with loading the model from FBX file into proper values

    Vector3f bboxMin;
    Vector3f bboxMax;

    // void debugBones();
    // void debugOffsetAccuracy();

};


class Animation {
public:
    Animation(const std::string &fbxPath, const GLuint shader);
    ~Animation();

    void animateAt(double time);    // Advance the skeleton to the given time (in seconds)
    void draw();                    // draw the skinned mesh (upload bone matrices to GPU) after interpolation

    Mesh* character;                // mesh + bone hierarchy
    double startTime = -1;

private:
    GLuint skinProgram;             // shader ID
    GLint locBoneMatrices;          // uniform location for uBoneMatrices[]
    GLuint locModel_;               // uniform location for model
    GLuint VAO;                     // VAO
    GLsizei indexCount;             // how many indices to draw

    // helpers
    void initMeshBuffers(const aiScene* scene);
};


#endif // ANIMATOR_H