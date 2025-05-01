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
// #include <set>
// #include "nanogui/glutil.h"
// #include "utils.h"


using namespace std;

void Bone::interpolateAt(double time, Matrix4f &parentTransform) {
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

// Mesh::Mesh()
//   : bones(new unordered_map<string, Bone*>()),
//     rootBone(nullptr),
//     vertices(nullptr),
//     duration(0.0),
//     VAO(0),
//     indexCount(0)
// {}

// Mesh::~Mesh() {
//     for (auto &p : *bones) delete p.second;
//     delete bones;
//     delete vertices;
// }

void Mesh::retrieveSceneValues(const aiScene* scene) {
    bones = new unordered_map<string, Bone*>();
    int curr = 0;
    for (int i = 0; i < scene->mNumMeshes; i++) {
        const aiMesh* currMesh = scene->mMeshes[i];
        for (int j = 0; j < currMesh->mNumBones; j++) {
            const aiBone* currBone = currMesh->mBones[j];
            Bone* b = new Bone();
            curr += 1;
            b->id = curr;
            b->name = currBone->mName.C_Str();
            b->offsetMatrix = Eigen::Map<Eigen::Matrix4f>((float*)currBone->mOffsetMatrix[0]).transpose();
            (*bones)[b->name] = b;
        }
    }

    // loading keyframe data for bones
    const aiAnimation* currAnimation = scene->mAnimations[0];
    duration = currAnimation->mDuration;
    double ticksPerSecond = currAnimation->mTicksPerSecond;

    for (int i = 0; i < currAnimation->mNumChannels; i++) {
        const aiNodeAnim* currChannel = currAnimation->mChannels[i];
        Bone* b = (*bones)[currChannel->mNodeName.C_Str()];

        b->positionKeys.resize(currChannel->mNumPositionKeys);
        b->positionTimes.resize(currChannel->mNumPositionKeys);
        for (int j = 0; j < currChannel->mNumPositionKeys; j++) {
            b->positionKeys[j] = Vector3f(currChannel->mPositionKeys[j].mValue.x, currChannel->mPositionKeys[j].mValue.y, currChannel->mPositionKeys[j].mValue.z);
            b->positionTimes[j] = currChannel->mPositionKeys[j].mTime;
        }

        b->rotationKeys.resize(currChannel->mNumRotationKeys);
        b->rotationTimes.resize(currChannel->mNumRotationKeys);
        for (int j = 0; j < currChannel->mNumRotationKeys; j++) {
            b->rotationKeys[j] = Eigen::Quaternionf(currChannel->mRotationKeys[j].mValue.w, currChannel->mRotationKeys[j].mValue.x, currChannel->mRotationKeys[j].mValue.y, currChannel->mRotationKeys[j].mValue.z);
            b->rotationTimes[j] = currChannel->mRotationKeys[j].mTime;
        }

        b->scalingKeys.resize(currChannel->mNumScalingKeys);
        b->scalingTimes.resize(currChannel->mNumScalingKeys);
        for (int j = 0; j < currChannel->mNumScalingKeys; j++) {
            b->scalingKeys[j] = Vector3f(currChannel->mScalingKeys[j].mValue.x, currChannel->mScalingKeys[j].mValue.y, currChannel->mScalingKeys[j].mValue.z);
            b->scalingTimes[j] = currChannel->mScalingKeys[j].mTime;
        }

        buildBoneHierarchy(scene->mRootNode, nullptr);
    }

}

void Mesh::buildBoneHierarchy(const aiNode* node, Bone* parent) {
    auto curr = bones->find(node->mName.C_Str());

    Bone* b = nullptr;
    if (curr != bones->end()) {
        b = curr->second;
    }
    if (b) {
        b->restLocalTransformation = Eigen::Map<Eigen::Matrix4f>((float*)node->mTransformation[0]).transpose();
        if (parent) {
            parent->children->push_back(b);
        } else {
            rootBone = b;
        }
        parent = b;
    }
    for (int i = 0; i < node->mNumChildren; i++) {
        buildBoneHierarchy(node->mChildren[i], parent);
    }
}


void Mesh::findFinalBoneMatrices(double time,vector<Eigen::Matrix4f>& boneMatrices) {
    if (!rootBone) {
        return;
    } 
    Matrix4f parentTrans = Eigen::Matrix4f::Identity();
    rootBone->interpolateAt(time, parentTrans);
    boneMatrices.clear();
    getBoneMatrices(rootBone, boneMatrices);
}

void Mesh::getBoneMatrices(Bone* bone, vector<Eigen::Matrix4f>& boneMatrices) {
    boneMatrices.push_back(bone->globalTransformation * bone->offsetMatrix);
    for (auto *c : *bone->children) {
        getBoneMatrices(c, boneMatrices);
    }
}

void Mesh::animateAt(double time) {
    findFinalBoneMatrices(time * ticksPerSecond, boneMatrices);
}

// Animation
Animation::Animation(nanogui::Screen *screen)
{
    skinProgram = createShaderProgram("./shaders/Default.vert", "./shaders/Default.frag");

}



void Animation::draw(vector<Matrix4f>* boneMatrices) {
    // WRONGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGG
    // glUseProgram(skinProgram);


    // // use shader program to update uniforms
    // GLint projLoc = glGetUniformLocation(skinProgram, "uProjection");
    // GLint viewLoc = glGetUniformLocation(skinProgram, "uView");
    // GLint boneLoc = glGetUniformLocation(skinProgram, "uBoneMatrices");

    // glUniformMatrix4fv(projLoc, 1, GL_FALSE, &projectionMatrix[0][0]);
    // glUniformMatrix4fv(viewLoc, 1, GL_FALSE, &viewMatrix[0][0]);
    // // upload all your bone transforms in one go:

    // float *ptr = boneMatrices.data()->data();

    // glUniformMatrix4fv(
    //     uBoneMatricesLoc,
    //     boneMatrices->size(),
    //     GL_FALSE,
    //     boneMatrices[0].data()
    // );

    // glBindVertexArray(character->VAO);
    // glDrawElements(GL_TRIANGLES,
    //                 character->indexCount,
    //                 GL_UNSIGNED_INT,
    //                 nullptr);

    // // 4) cleanup
    // glBindVertexArray(0);
    // glUseProgram(0);
    
}


void Animation::setupDrawCallback() {
    screen->setDrawContentsCallback([this]() {
        double now = glfwGetTime();
        // first frame
        if (startTime < 0.0) {
            startTime = now;
        }

        double time = now - startTime;
        this->character->animateAt(time);
        vector<Eigen::Matrix4f>* boneMatrices = &this->character->boneMatrices;
        draw(boneMatrices);
    });
}