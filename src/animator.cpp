#include "animator.h"
#include "Eigen/src/Geometry/Quaternion.h"
#include "GLFW/glfw3.h"
#include "nanogui/common.h"
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>
#include "shader.h"
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>


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

Mesh::Mesh()
  : bones(new unordered_map<string, Bone*>()),
    rootBone(nullptr),
    vertices(nullptr),
    duration(0.0),
    VAO(0),
    indexCount(0)
{}

Mesh::~Mesh() {
    for (auto &p : *bones) delete p.second;
    delete bones;
    delete vertices;
}

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
    
    // getBoneMatrices(rootBone, boneMatrices);

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


// void Mesh::findFinalBoneMatrices(double time,vector<Eigen::Matrix4f>& boneMatrices) {
//     // interpolates at the given time and updates boneMatrices with the new bone matrices
//     if (!rootBone) {
//         return;
//     } 
//     Matrix4f parentTrans = Eigen::Matrix4f::Identity();
//     rootBone->interpolateAt(time, parentTrans);
//     boneMatrices.clear();
//     getBoneMatrices(rootBone, boneMatrices);
// }

// potential buggy point if bone matrices are not in the same order
void Mesh::getBoneMatrices(Bone* bone, vector<Eigen::Matrix4f>& boneMatrices) {
    // retreives the bone matrices recursively and 
    boneMatrices.push_back(bone->globalTransformation * bone->offsetMatrix);
    for (auto *c : *bone->children) {
        getBoneMatrices(c, boneMatrices);
    }
}


void Mesh::animateAt(double time) {
    // findFinalBoneMatrices(time * ticksPerSecond, boneMatrices);
    if (!rootBone) {
        return;
    } 
    Matrix4f parentTrans = Eigen::Matrix4f::Identity();
    rootBone->interpolateAt(time, parentTrans);
    boneMatrices.clear();
    getBoneMatrices(rootBone, boneMatrices);
}



//----------------------------------------------------------------------
// Constructor: compile skin‐shader, load FBX into `character`, build VAO/EBO
//----------------------------------------------------------------------
Animation::Animation(const std::string &fbxPath, const GLuint shader) {
    skinProgram = shader;
    locBoneMatrices = glGetUniformLocation(skinProgram, "uBoneMatrices");


    // 2) load your model via Assimp into Mesh*
    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(fbxPath, aiProcess_Triangulate | aiProcess_GenSmoothNormals | aiProcess_JoinIdenticalVertices);
    if (!scene) {
        std::cerr << "FBX load error: " << importer.GetErrorString() << std::endl;
        character = nullptr;
        return;
    }
    character = new Mesh();
    character->retrieveSceneValues(scene);

    // 3) build the VAO/VBO/EBO from character->vertices and your index list
    initMeshBuffers(scene);

    // glEnable(GL_DEPTH_TEST);
}


Bone::~Bone() {

}

Animation::~Animation() {
    delete character;
    glDeleteProgram(skinProgram);
    glDeleteVertexArrays(1, &VAO);
    // (also delete any VBO/EBO you generated if you stored them)
}


//----------------------------------------------------------------------
// Build a VAO that matches your Vertex struct & index array
//----------------------------------------------------------------------
void Animation::initMeshBuffers(const aiScene* scene) {
    // assume your Mesh has:
    //   std::vector<Vertex> *vertices;
    //   std::vector<unsigned int>   indices;   // you must fill this
    auto &verts = *character->vertices;
    std::vector<unsigned int> indices;
    // TODO: fill 'indices' from your aiMesh data (e.g. loop over scene->mMeshes)

    for (unsigned m = 0; m < scene->mNumMeshes; ++m) {
        const aiMesh* mesh = scene->mMeshes[m];
        for (unsigned f = 0; f < mesh->mNumFaces; ++f) {
            const aiFace& face = mesh->mFaces[f];
            if (face.mNumIndices != 3) continue;  // only triangles
            indices.push_back(face.mIndices[0]);
            indices.push_back(face.mIndices[1]);
            indices.push_back(face.mIndices[2]);
        }
    }

    indexCount = (GLsizei)indices.size();

    GLuint VBO, EBO;
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);

    glBindVertexArray(VAO);

    // upload vertex data
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER,
                 verts.size() * sizeof(Vertex),
                 verts.data(),
                 GL_STATIC_DRAW);

    // upload index data
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                 indices.size() * sizeof(unsigned int),
                 indices.data(),
                 GL_STATIC_DRAW);

    // set up layout to match your default.vert:
    //   layout(0) in vec3 in_position;
    //   layout(1) in vec3 in_normal;
    //   layout(2) in vec3 in_color;
    //   (and, if you have boneIndices/weights, pick locations 3 and 4)
    constexpr GLsizei S = sizeof(Vertex);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, S,
                          (void*)offsetof(Vertex, position));

    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, S,
                          (void*)offsetof(Vertex, normal));

    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, S,
                          (void*)offsetof(Vertex, color));

    // …if you have boneIndices (as GLuint[4]) and weights (float[4]):
    glEnableVertexAttribArray(3);
    glVertexAttribIPointer(3, 4, GL_UNSIGNED_INT, S,
                           (void*)offsetof(Vertex, boneIndices));

    glEnableVertexAttribArray(4);
    glVertexAttribPointer(4, 4, GL_FLOAT, GL_FALSE, S,
                          (void*)offsetof(Vertex, weights));

    glBindVertexArray(0);

    // ####### BONE MATRIX HANDLING ########
    this->draw();

}


//----------------------------------------------------------------------
// Advance your mesh’s bone matrices
//----------------------------------------------------------------------
void Animation::animateAt(double time) {
    if (character) {
        character->animateAt(time);
    }
}


// Draw the skinned mesh: upload bone array, bind VAO, issue draw
void Animation::draw() {
    if (!character) return;


    // assume the caller already did glUseProgram(skinProgram),
    // and uploaded uM, uV, uP.
    const auto &bones = character->boneMatrices;
    glUniformMatrix4fv(locBoneMatrices,
                        (GLsizei)bones.size(),
                        GL_FALSE,
                        bones.data()->data());
    
    glBindVertexArray(VAO);
    glDrawElements(GL_TRIANGLES, indexCount, GL_UNSIGNED_INT, nullptr);
    glBindVertexArray(0);
    glUseProgram(0);
    
}


