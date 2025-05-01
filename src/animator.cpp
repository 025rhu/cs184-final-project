#include "animator.h"
#include "Eigen/src/Geometry/Quaternion.h"
#include "GLFW/glfw3.h"
#include "nanogui/common.h"
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>
// #include <glad/glad.h>
// #include "CGL/CGL.h"
// #include <CGL/vector3D.h>
// #include <iostream>
// #include <set>
// #include <string>
// #include "nanogui/glutil.h"
// #include "utils.h"


using namespace std;

void Bone::interpolateAt(double time, Matrix4f parentTransform) {
    Matrix4f localTransform = buildLocalTransform(time);
    Matrix4f globalTransform = localTransform * parentTransform;    // try parent * local if this looks wonky
    this->localTransformation = localTransform;
    this->globalTransformation = globalTransform;
    
    for (auto child : *children) {
        child->interpolateAt(time, globalTransform);
    }

}

Vector3f lerp(Vector3f p0, Vector3f p1, double t) {
    return (1 - t) * p0 + (t * p1);
}

Vector3f Bone::interpolatePosition(double time) const {
    int i = findPositionIndex(time);

    double t0 = positionTimes[i];
    double t1 = positionTimes[i + 1];
    Vector3f p0 = positionKeys[i];
    Vector3f p1 = positionKeys[i + 1];

    double t = (time - t0) / (t1 - t0);
    Vector3f position = lerp(p0, p1, t);
    return position;
}

Eigen::Quaternionf Bone::interpolateRotation(double time) const {
    int i = findRotationIndex(time);
    
    double t0 = rotationTimes[i];
    double t1 = rotationTimes[i + 1];
    Eigen::Quaternionf q0 = rotationKeys[i];
    Eigen::Quaternionf q1 = rotationKeys[i + 1];

    double t = (time - t0) / (t1 - t0);
    Eigen::Quaternionf rotation = q0.slerp(t, q1);
    return rotation;
}

Vector3f Bone::interpolateScaling(double time) const {
    int i = findScalingIndex(time);
    
    double t0 = scalingTimes[i];
    double t1 = scalingTimes[i + 1];
    Vector3f p0 = scalingKeys[i];
    Vector3f p1 = scalingKeys[i + 1];

    double t = (time - t0) / (t1 - t0);
    Vector3f scale = lerp(p0, p1, t);
    return scale;
}

int Bone::findPositionIndex(double time) const {
    if (positionTimes.empty()) {
        std::cout << "Error: Position times empty." << std::endl;
        return -1;
    }
    if (time <= positionTimes[0]) {
        return 0;
    }
    if (time >= positionTimes.back()){
        return positionTimes.size() - 2;
    }
    for (int i = 0; i < positionTimes.size() - 1; i++) {
        if (time >= positionTimes[i] && time < positionTimes[i + 1]) {
            return i;
        }   
    }
    std::cout << "Time stamp out of range for positions." << std::endl;
    return -1;
}

int Bone::findRotationIndex(double time) const {
    if (rotationTimes.empty()) {
        std::cout << "Error: Position times empty." << std::endl;
        return -1;
    }
    if (time <= rotationTimes[0]) {
        return 0;
    }
    if (time >= rotationTimes.back()){
        return rotationTimes.size() - 2;
    }
    for (int i = 0; i < rotationTimes.size() - 1; i++) {
        if (time >= rotationTimes[i] && time < rotationTimes[i + 1]) {
            return i;
        }
    }
    std::cout << "Time stamp out of range for rotations." << std::endl;
    return -1;
}

int Bone::findScalingIndex(double time) const {
    if (scalingTimes.empty()) {
        std::cout << "Error: Position times empty." << std::endl;
        return -1;
    }
    if (time <= scalingTimes[0]) {
        return 0;
    }
    if (time >= scalingTimes.back()){
        return scalingTimes.size() - 2;
    }
    for (int i = 0; i < scalingTimes.size() - 1; i++) {
        if (time >= scalingTimes[i] && time < scalingTimes[i + 1]) {
            return i;
        }
    }
    std::cout << "Time stamp out of range for scaling." << std::endl;
    return -1;
}

Matrix4f Bone::buildLocalTransform(double time) {     
    Eigen::Vector3f posTransform = interpolatePosition(time);
    Eigen::Quaternionf rotateTransform = interpolateRotation(time);
    Eigen::Vector3f scaleTransform = interpolateScaling(time);

    return (Eigen::Translation3f(posTransform) * rotateTransform * Eigen::Scaling(scaleTransform)).matrix();
}



// void Mesh::retrieveSceneValues(const aiScene* scene) {
//     bones = new unordered_map<string, Bone*>();
//     for (int i = 0; i < scene->mNumMeshes; i++) {
//         const aiMesh* currMesh = scene->mMeshes[i];
//         for (int j = 0; j < currMesh->mNumBones; j++) {
//             const aiBone* currBone = currMesh->mBones[j];
//             Bone* b = new Bone();
//             b->name = currBone->mName;
//             b->offsetMatrix = Map<Matrix4f>((float*)currBone->mOffsetMatrix[0]).transpose();
//             (*bones)[b->name] = b;
//         }
//     }

//     // loading keyframe data for bones
//     const aiAnimation* currAnimation = scene->mAnimations[0];
//     duration = currAnimation->duration;
//     // todo: set other attributes of animation?

//     for (int i = 0; i < currAnimation->mNumChannels; i++) {
//         const aiNodeAnim* currChannel = currAnimation->mChannels[i];
//         Bone* b = (*bones)[currChannel->mNodeName];

//         b->positionKeys.resize(currChannel->mNumPositionKeys);
//         b->positionTimes.resize(currChannel->mNumPositionKeys);
//         for (int j = 0; j < currChannel->mNumPositionKeys; j++) {
//             b->positionKeys[j] = Vector3f(currChannel->mPositionKeys[j].mValue.x, currChannel->mPositionKeys[j].mValue.y, currChannel->mPositionKeys[j].mValue.z);
//             b->positionTimes[j] = currChannel->mPositionKeys[j].mTime;
//         }

//         b->rotationKeys.resize(currChannel->mNumRotationKeys);
//         b->rotationTimes.resize(currChannel->mNumRotationKeys);
//         for (int j = 0; j < currChannel->mNumRotationKeys; j++) {
//             b->rotationKeys[j] = Eigen::Quaternionf(currChannel->mRotationKeys[j].mValue.w, currChannel->mRotationKeys[j].mValue.x, currChannel->mRotationKeys[j].mValue.y, currChannel->mRotationKeys[j].mValue.z);
//             b->rotationTimes[j] = currChannel->mRotationKeys[j].mTime;
//         }

//         b->scalingKeys.resize(currChannel->mNumScalingKeys);
//         b->scalingTimes.resize(currChannel->mNumScalingKeys);
//         for (int j = 0; j < currChannel->mNumScalingKeys; j++) {
//             b->scalingKeys[j] = Vector3f(currChannel->mNumScalingKeys[j].mValue.x, currChannel->mNumScalingKeys[j].mValue.y, currChannel->mNumScalingKeys[j].mValue.z);
//             b->scalingTimes[j] = currChannel->mNumScalingKeys[j].mTime;
//         }
//     }


// }



// // finding final bone matrices to help with skinning for animation
vector<Matrix4f>* Mesh::getBoneMatrices() {
    vector<Matrix4f>* boneMatrices = new vector<Matrix4f>;
    for (int i = 0; i < bones.size(); i ++) {
        boneMatrices[i] = {bones[i]->globalTransformation * bones[i]->offsetMatrix};
    }
    return boneMatrices;
}


void Animation::draw(vector<Matrix4f>* boneMatrices) {
    // glUseProgram(shader);

    // Update uBoneMatrices before drawing
    // glUniformMatrix4fv(
    //     uBoneMatricesLoc,
    //     boneMatrices->size(),
    //     GL_FALSE,
    //     boneMatrices[0].data()
    // );

}


void Animation::setupDrawCallback() {
    double now = glfwGetTime();
    // first frame
    if (startTime < 0.0) {
        startTime = now;
    }

    double time = now - startTime;
    this->character->animateAt(time);
    vector<Matrix4f>* boneMatrices = this->character->getBoneMatrices();
    draw(boneMatrices);
}