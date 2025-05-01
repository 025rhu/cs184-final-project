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
#include "shader.h"

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

    int id;

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

    int findPositionIndex(double time) const;
    int findRotationIndex(double time) const;
    int findScalingIndex(double time) const;

    Matrix4f buildLocalTransform(double time);

};


struct Mesh {
    // mesh architecture
    // std::vector<Bone*> bones;
    std::vector<Eigen::Matrix4f> boneMatrices;
    unordered_map<string, Bone*>* bones;
    Bone* rootBone;
    vector<Vertex>* vertices;
    GLuint VAO;
    GLuint indexCount;
    double duration = 0.0;
    double ticksPerSecond = 25.0;

    Mesh();
    ~Mesh();

    void animateAt(double time);
    // vector<Matrix4f>* getBoneMatrices();
    void retrieveSceneValues(const aiScene* scene);
    void findFinalBoneMatrices(double time, vector<Eigen::Matrix4f>& boneMatrices);
    
private:
    void getBoneMatrices(Bone* bone, vector<Eigen::Matrix4f>& boneMatrices);
    void buildBoneHierarchy(const aiNode* node, Bone* parent);

};


class Animation {
public:
    Animation(const std::string &fbxPath);
    ~Animation();

    // Advance the skeleton to the given time (in seconds)
    void animateAt(double time);

    // Draw the skinned mesh.  
    // Caller must have already done:
    //    glUseProgram(shaderID);
    //    glUniformMatrix4fv(uM), uV, uP
    // before invoking draw().
    void draw(const std::vector<Eigen::Matrix4f> &boneMatrices);

    Mesh* character;      // your mesh + bone hierarchy
    double startTime = -1;

private:
    // skin‐shader program
    GLuint skinProgram;
    GLint loc_uBones;     // uniform location for uBoneMatrices[]

    // VAO + how many indices to draw
    GLuint VAO;
    GLsizei indexCount;

    // Helpers
    void initShader(const std::string &vs = "shaders/Default.vert",
                    const std::string &fs = "shaders/Default.frag");
    void initMeshBuffers();
};


#endif // ANIMATOR_H