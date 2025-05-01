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

Bone::Bone() : id(-1) {
    offsetMatrix.setIdentity();
    restLocalTransformation.setIdentity();
    localTransformation.setIdentity();
    globalTransformation.setIdentity();
}

void Bone::interpolateAt(double time, Matrix4f &parentTransform) {
    Matrix4f localTransform = buildLocalTransform(time);
    Matrix4f globalTransform = localTransform * parentTransform;    // try parent * local if this looks wonky
    this->localTransformation = localTransform;
    this->globalTransformation = globalTransform;
    
    for (auto child : children) {
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

// void Mesh::retrieveSceneValues(const aiScene* scene) {
//     std::cout << "[Info] Starting scene parsing..." << std::endl;

//     if (!scene || !scene->HasMeshes() || !scene->HasAnimations()) {
//         std::cerr << "[Error] Scene has no meshes or animations." << std::endl;
//         return;
//     }

//     bones = new std::unordered_map<std::string, Bone*>();
//     std::unordered_map<std::string, Bone*>& boneMap = *bones;

//     // STEP 1: Create bones from hierarchy
//     int nextBoneId = 0;
//     std::function<void(const aiNode*, Bone*)> buildHierarchy;

//     buildHierarchy = [&](const aiNode* node, Bone* parent) {
//         std::string name = node->mName.C_Str();
    
//         Bone* bone;
//         if (boneMap.count(name)) {
//             bone = boneMap[name];
//         } else {
//             bone = new Bone();
//             bone->name = name;
//             boneMap[name] = bone;
//         }
    
//         // Assign a unique ID in hierarchy order
//         if (bone->id == -1) {
//             bone->id = nextBoneId++;
//         }
    
//         bone->restLocalTransformation = Eigen::Map<const Matrix4f>((float*)&node->mTransformation).transpose();
    
//         if (parent) {
//             parent->children.push_back(bone);
//         } else {
//             rootBone = bone;
//         }
    
//         for (unsigned int i = 0; i < node->mNumChildren; ++i) {
//             buildHierarchy(node->mChildren[i], bone);
//         }
//     };
    
//     buildHierarchy(scene->mRootNode, nullptr);

//     // STEP 2: Load animation keyframes
//     const aiAnimation* anim = scene->mAnimations[0];
//     duration = anim->mDuration;
//     double ticksPerSecond = anim->mTicksPerSecond != 0.0 ? anim->mTicksPerSecond : 25.0;

//     for (unsigned int i = 0; i < anim->mNumChannels; ++i) {
//         const aiNodeAnim* channel = anim->mChannels[i];
//         std::string name = channel->mNodeName.C_Str();

//         Bone* bone;
//         if (boneMap.count(name)) {
//             bone = boneMap[name];
//         } else {
//             bone = new Bone();
//             bone->name = name;
//             boneMap[name] = bone;
//         }

//         for (unsigned int j = 0; j < channel->mNumPositionKeys; ++j) {
//             bone->positionTimes.push_back(channel->mPositionKeys[j].mTime);
//             bone->positionKeys.emplace_back(
//                 channel->mPositionKeys[j].mValue.x,
//                 channel->mPositionKeys[j].mValue.y,
//                 channel->mPositionKeys[j].mValue.z);
//         }

//         for (unsigned int j = 0; j < channel->mNumRotationKeys; ++j) {
//             bone->rotationTimes.push_back(channel->mRotationKeys[j].mTime);
//             bone->rotationKeys.emplace_back(
//                 channel->mRotationKeys[j].mValue.w,
//                 channel->mRotationKeys[j].mValue.x,
//                 channel->mRotationKeys[j].mValue.y,
//                 channel->mRotationKeys[j].mValue.z);
//         }

//         for (unsigned int j = 0; j < channel->mNumScalingKeys; ++j) {
//             bone->scalingTimes.push_back(channel->mScalingKeys[j].mTime);
//             bone->scalingKeys.emplace_back(
//                 channel->mScalingKeys[j].mValue.x,
//                 channel->mScalingKeys[j].mValue.y,
//                 channel->mScalingKeys[j].mValue.z);
//         }
//     }

//     // STEP 3: Add offset matrices from mesh->bones[]
//     for (unsigned int meshIdx = 0; meshIdx < scene->mNumMeshes; ++meshIdx) {
//         const aiMesh* mesh = scene->mMeshes[meshIdx];
//         for (unsigned int i = 0; i < mesh->mNumBones; ++i) {
//             const aiBone* aiB = mesh->mBones[i];
//             std::string name = aiB->mName.C_Str();

//             if (!boneMap.count(name)) {
//                 Bone* bone = new Bone();
//                 bone->name = name;
//                 boneMap[name] = bone;
//             }

//             boneMap[name]->offsetMatrix = Eigen::Map<const Matrix4f>((float*)&aiB->mOffsetMatrix).transpose();
//         }
//     }

//     std::cout << "[Info] Finished building bone hierarchy and animation data." << std::endl;
// }

void Mesh::retrieveSceneValues(const aiScene* scene) {
    std::cout << "[Info] Starting scene parsing..." << std::endl;

    if (!scene || !scene->HasMeshes() || !scene->HasAnimations()) {
        std::cerr << "[Error] Scene has no meshes or animations." << std::endl;
        return;
    }

    bones = new std::unordered_map<std::string, Bone*>();
    std::unordered_map<std::string, Bone*>& boneMap = *bones;

    // Step 0: Only add bones referenced by the mesh
    std::unordered_set<std::string> usedBoneNames;
    for (unsigned int m = 0; m < scene->mNumMeshes; ++m) {
        const aiMesh* mesh = scene->mMeshes[m];
        for (unsigned int b = 0; b < mesh->mNumBones; ++b) {
            usedBoneNames.insert(mesh->mBones[b]->mName.C_Str());
        }
    }

    // Step 1: Create bones *only if animated AND used*
    const aiAnimation* anim = scene->mAnimations[0];
    duration = anim->mDuration;
    double ticksPerSecond = anim->mTicksPerSecond != 0.0 ? anim->mTicksPerSecond : 25.0;

    for (unsigned int i = 0; i < anim->mNumChannels; ++i) {
        const aiNodeAnim* channel = anim->mChannels[i];
        std::string name = channel->mNodeName.C_Str();

        // Skip bones that don’t affect the mesh
        if (!usedBoneNames.count(name)) continue;

        Bone* bone = new Bone();
        bone->name = name;

        for (unsigned int j = 0; j < channel->mNumPositionKeys; ++j) {
            bone->positionTimes.push_back(channel->mPositionKeys[j].mTime);
            bone->positionKeys.emplace_back(
                channel->mPositionKeys[j].mValue.x,
                channel->mPositionKeys[j].mValue.y,
                channel->mPositionKeys[j].mValue.z);
        }

        for (unsigned int j = 0; j < channel->mNumRotationKeys; ++j) {
            bone->rotationTimes.push_back(channel->mRotationKeys[j].mTime);
            bone->rotationKeys.emplace_back(
                channel->mRotationKeys[j].mValue.w,
                channel->mRotationKeys[j].mValue.x,
                channel->mRotationKeys[j].mValue.y,
                channel->mRotationKeys[j].mValue.z);
        }

        for (unsigned int j = 0; j < channel->mNumScalingKeys; ++j) {
            bone->scalingTimes.push_back(channel->mScalingKeys[j].mTime);
            bone->scalingKeys.emplace_back(
                channel->mScalingKeys[j].mValue.x,
                channel->mScalingKeys[j].mValue.y,
                channel->mScalingKeys[j].mValue.z);
        }

        boneMap[name] = bone;
    }

    // Step 2: Add offset matrices to those already in boneMap
    for (unsigned int m = 0; m < scene->mNumMeshes; ++m) {
        const aiMesh* mesh = scene->mMeshes[m];
        for (unsigned int i = 0; i < mesh->mNumBones; ++i) {
            const aiBone* aiB = mesh->mBones[i];
            std::string name = aiB->mName.C_Str();

            if (boneMap.count(name)) {
                boneMap[name]->offsetMatrix = Eigen::Map<const Matrix4f>((float*)&aiB->mOffsetMatrix).transpose();
            }
        }
    }

    // Step 3: Build hierarchy (only for bones in boneMap)
    int nextBoneId = 0;
    std::function<void(const aiNode*, Bone*)> buildHierarchy;
    buildHierarchy = [&](const aiNode* node, Bone* parent) {
        std::string name = node->mName.C_Str();
        if (!boneMap.count(name)) return; // skip unused bones

        Bone* bone = boneMap[name];
        bone->id = nextBoneId++;
        bone->restLocalTransformation = Eigen::Map<const Matrix4f>((float*)&node->mTransformation).transpose();

        if (parent) {
            parent->children.push_back(bone);
        } else {
            rootBone = bone;
        }

        for (unsigned int i = 0; i < node->mNumChildren; ++i) {
            buildHierarchy(node->mChildren[i], bone);
        }
    };

    buildHierarchy(scene->mRootNode, nullptr);

    std::cout << "[Info] Finished building bone hierarchy and animation data." << std::endl;
}



// void Mesh::retrieveSceneValues(const aiScene* scene) {
//     std::cout << "entered scene values" << std::endl;

//     bones = new unordered_map<string, Bone*>();
//     int curr = 0;
//     for (int i = 0; i < scene->mNumMeshes; i++) {
//         const aiMesh* currMesh = scene->mMeshes[i];
//         for (int j = 0; j < currMesh->mNumBones; j++) {
//             const aiBone* currBone = currMesh->mBones[j];
//             Bone* b = new Bone();
//             curr += 1;
//             b->id = curr;
//             b->name = currBone->mName.C_Str();

//             b->offsetMatrix = Eigen::Map<Eigen::Matrix4f>((float*)currBone->mOffsetMatrix[0]).transpose();
//             (*bones)[b->name] = b;
//             std::cout << "added bone " << b->name << " to bone map" << std::endl;

//         }
//     }
//     std::cout << "finished first loop" << std::endl;

//     // loading keyframe data for bones
//     const aiAnimation* currAnimation = scene->mAnimations[0];
//     duration = currAnimation->mDuration;
//     double ticksPerSecond = currAnimation->mTicksPerSecond;
//     std::cout << "starting second loop" << std::endl;

//     for (int i = 0; i < currAnimation->mNumChannels; i++) {
//         const aiNodeAnim* currChannel = currAnimation->mChannels[i];
//         auto it = bones->find(currChannel->mNodeName.C_Str());
//         if (it == bones->end()) {
//             std::cerr << "Bone not found in mesh: " << currChannel->mNodeName.C_Str() << std::endl;
//             return;  // skip this channel
//         }
//         Bone* b = (*bones)[currChannel->mNodeName.C_Str()];
//         std::cout << "created bone pointer" << std::endl;

//         b->positionKeys.resize(currChannel->mNumPositionKeys);
//         b->positionTimes.resize(currChannel->mNumPositionKeys);
//         std::cout << "resized position keys and times" << std::endl;

//         for (int j = 0; j < currChannel->mNumPositionKeys; j++) {
//             b->positionKeys[j] = Vector3f(currChannel->mPositionKeys[j].mValue.x, currChannel->mPositionKeys[j].mValue.y, currChannel->mPositionKeys[j].mValue.z);
//             b->positionTimes[j] = currChannel->mPositionKeys[j].mTime;
//         }
//         std::cout << "finished first INNER loop" << std::endl;

//         b->rotationKeys.resize(currChannel->mNumRotationKeys);
//         b->rotationTimes.resize(currChannel->mNumRotationKeys);
//         for (int j = 0; j < currChannel->mNumRotationKeys; j++) {
//             b->rotationKeys[j] = Eigen::Quaternionf(currChannel->mRotationKeys[j].mValue.w, currChannel->mRotationKeys[j].mValue.x, currChannel->mRotationKeys[j].mValue.y, currChannel->mRotationKeys[j].mValue.z);
//             b->rotationTimes[j] = currChannel->mRotationKeys[j].mTime;
//         }
//         std::cout << "finished second INNER loop" << std::endl;


//         b->scalingKeys.resize(currChannel->mNumScalingKeys);
//         b->scalingTimes.resize(currChannel->mNumScalingKeys);
//         for (int j = 0; j < currChannel->mNumScalingKeys; j++) {
//             b->scalingKeys[j] = Vector3f(currChannel->mScalingKeys[j].mValue.x, currChannel->mScalingKeys[j].mValue.y, currChannel->mScalingKeys[j].mValue.z);
//             b->scalingTimes[j] = currChannel->mScalingKeys[j].mTime;
//         }
//         std::cout << "finished third INNER loop" << std::endl;

//         std::cout << "calling build bone hierarchy" << std::endl;

//         buildBoneHierarchy(scene->mRootNode, nullptr);
//     }
//     std::cout << "finished second loop" << std::endl;

//     // getBoneMatrices(rootBone, boneMatrices);

// }

void Mesh::buildBoneHierarchy(const aiNode* node, Bone* parent) {
    std::cout << "entered build bone hierarchy" << std::endl;

    auto curr = bones->find(node->mName.C_Str());

    Bone* b = nullptr;
    if (curr != bones->end()) {
        b = curr->second;
    }
    if (b) {
        b->restLocalTransformation = Eigen::Map<Eigen::Matrix4f>((float*)node->mTransformation[0]).transpose();
        if (parent) {
            parent->children.push_back(b);
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
    for (auto *c : bone->children) {
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
    cout << "created new mesh" << endl;
    character->retrieveSceneValues(scene);
    cout << "Retrieved scene values" << endl;

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
    // // assume your Mesh has:
    // //   std::vector<Vertex> *vertices;
    // //   std::vector<unsigned int>   indices;   // you must fill this
    // auto &verts = *character->vertices;
    // std::vector<unsigned int> indices;
    // // TODO: fill 'indices' from your aiMesh data (e.g. loop over scene->mMeshes)

    // for (unsigned m = 0; m < scene->mNumMeshes; ++m) {
    //     const aiMesh* mesh = scene->mMeshes[m];
    //     for (unsigned f = 0; f < mesh->mNumFaces; ++f) {
    //         const aiFace& face = mesh->mFaces[f];
    //         if (face.mNumIndices != 3) continue;  // only triangles
    //         indices.push_back(face.mIndices[0]);
    //         indices.push_back(face.mIndices[1]);
    //         indices.push_back(face.mIndices[2]);
    //     }
    // }

    // indexCount = (GLsizei)indices.size();

    // GLuint VBO, EBO;
    // glGenVertexArrays(1, &VAO);
    // glGenBuffers(1, &VBO);
    // glGenBuffers(1, &EBO);

    // glBindVertexArray(VAO);

    // // upload vertex data
    // glBindBuffer(GL_ARRAY_BUFFER, VBO);
    // glBufferData(GL_ARRAY_BUFFER,
    //              verts.size() * sizeof(Vertex),
    //              verts.data(),
    //              GL_STATIC_DRAW);

    // // upload index data
    // glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    // glBufferData(GL_ELEMENT_ARRAY_BUFFER,
    //              indices.size() * sizeof(unsigned int),
    //              indices.data(),
    //              GL_STATIC_DRAW);

    // // set up layout to match your default.vert:
    // //   layout(0) in vec3 in_position;
    // //   layout(1) in vec3 in_normal;
    // //   layout(2) in vec3 in_color;
    // //   (and, if you have boneIndices/weights, pick locations 3 and 4)
    // constexpr GLsizei S = sizeof(Vertex);
    // glEnableVertexAttribArray(0);
    // glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, S,
    //                       (void*)offsetof(Vertex, position));

    // glEnableVertexAttribArray(1);
    // glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, S,
    //                       (void*)offsetof(Vertex, normal));

    // glEnableVertexAttribArray(2);
    // glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, S,
    //                       (void*)offsetof(Vertex, color));

    // // …if you have boneIndices (as GLuint[4]) and weights (float[4]):
    // glEnableVertexAttribArray(3);
    // glVertexAttribIPointer(3, 4, GL_UNSIGNED_INT, S,
    //                        (void*)offsetof(Vertex, boneIndices));

    // glEnableVertexAttribArray(4);
    // glVertexAttribPointer(4, 4, GL_FLOAT, GL_FALSE, S,
    //                       (void*)offsetof(Vertex, weights));

    // glBindVertexArray(0);
    std::vector<Vertex> verts;
    std::vector<unsigned int> indices;

    for (unsigned m = 0; m < scene->mNumMeshes; ++m) {
        const aiMesh* mesh = scene->mMeshes[m];

        // Load vertices
        for (unsigned int v = 0; v < mesh->mNumVertices; ++v) {
            Vertex vert;

            // Position
            vert.position = Eigen::Vector3f(
                mesh->mVertices[v].x,
                mesh->mVertices[v].y,
                mesh->mVertices[v].z
            );

            // Normal
            if (mesh->HasNormals()) {
                vert.normal = Eigen::Vector3f(
                    mesh->mNormals[v].x,
                    mesh->mNormals[v].y,
                    mesh->mNormals[v].z
                );
            } else {
                vert.normal = Eigen::Vector3f(0, 0, 0);
            }

            // Color (optional: here defaulted)
            vert.color = Eigen::Vector3f(1.0f, 1.0f, 1.0f);

            // Bone weights (optional default)
            vert.boneIndices.resize(4, 0);  // Assuming up to 4 bones per vertex
            vert.weights.resize(4, 0.0f);

            verts.push_back(vert);
        }

        // Fill bone weights
        for (unsigned int b = 0; b < mesh->mNumBones; ++b) {
            const aiBone* bone = mesh->mBones[b];
            std::string boneName = bone->mName.C_Str();

            int boneId = 0;
            if (character->bones->count(boneName)) {
                boneId = (*character->bones)[boneName]->id;
            }

            for (unsigned int w = 0; w < bone->mNumWeights; ++w) {
                unsigned int vertexId = bone->mWeights[w].mVertexId;
                float weight = bone->mWeights[w].mWeight;

                Vertex& v = verts[vertexId];
                for (int i = 0; i < 4; ++i) {
                    if (v.weights[i] == 0.0f) {
                        v.boneIndices[i] = boneId;
                        v.weights[i] = weight;
                        break;
                    }
                }
            }
        }

        // Load indices
        for (unsigned f = 0; f < mesh->mNumFaces; ++f) {
            const aiFace& face = mesh->mFaces[f];
            if (face.mNumIndices != 3) continue;
            indices.push_back(face.mIndices[0]);
            indices.push_back(face.mIndices[1]);
            indices.push_back(face.mIndices[2]);
        }
    }

    indexCount = (GLsizei)indices.size();

    // ---- Upload to GPU ----
    GLuint VBO, EBO;
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);
    glBindVertexArray(VAO);

    // Upload vertices
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, verts.size() * sizeof(Vertex), verts.data(), GL_STATIC_DRAW);

    // Upload indices
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.data(), GL_STATIC_DRAW);

    // Attribute pointers
    constexpr GLsizei S = sizeof(Vertex);
    glEnableVertexAttribArray(0); // position
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, S, (void*)offsetof(Vertex, position));

    glEnableVertexAttribArray(1); // normal
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, S, (void*)offsetof(Vertex, normal));

    glEnableVertexAttribArray(2); // color
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, S, (void*)offsetof(Vertex, color));

    glEnableVertexAttribArray(3); // bone indices
    glVertexAttribIPointer(3, 4, GL_UNSIGNED_INT, S, (void*)offsetof(Vertex, boneIndices));

    glEnableVertexAttribArray(4); // weights
    glVertexAttribPointer(4, 4, GL_FLOAT, GL_FALSE, S, (void*)offsetof(Vertex, weights));

    glBindVertexArray(0);

    // ####### BONE MATRIX HANDLING ########
    this->draw();
    cout << "finished initializing buffers" << endl;

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


