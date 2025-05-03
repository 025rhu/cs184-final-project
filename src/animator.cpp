#include "animator.h"
#include "Eigen/src/Geometry/Quaternion.h"
#include "nanogui/common.h"
#include <cfloat>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>
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
    Matrix4f globalTransform = parentTransform * localTransform;
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
    int i = findIndex(time, &positionTimes);

    double t0 = positionTimes[i];
    double t1 = positionTimes[i + 1];
    Vector3f p0 = positionKeys[i];
    Vector3f p1 = positionKeys[i + 1];

    double t = (time - t0) / (t1 - t0);
    Vector3f position = lerp(p0, p1, t);
    return position;
}

Eigen::Quaternionf Bone::interpolateRotation(double time) const {
    int i = findIndex(time, &rotationTimes);
    
    double t0 = rotationTimes[i];
    double t1 = rotationTimes[i + 1];
    Eigen::Quaternionf q0 = rotationKeys[i];
    Eigen::Quaternionf q1 = rotationKeys[i + 1];

    double t = (time - t0) / (t1 - t0);
    Eigen::Quaternionf rotation = q0.slerp(t, q1);
    return rotation;
}

Vector3f Bone::interpolateScaling(double time) const {
    int i = findIndex(time, &scalingTimes);
    
    double t0 = scalingTimes[i];
    double t1 = scalingTimes[i + 1];
    Vector3f p0 = scalingKeys[i];
    Vector3f p1 = scalingKeys[i + 1];

    double t = (time - t0) / (t1 - t0);
    Vector3f scale = lerp(p0, p1, t);
    return scale;
}

int Bone::findIndex(double time, const vector<double>* times) const {
    if (times->empty()) {
        std::cout << "Error: Keyframe times empty." << std::endl;
        return -1;
    }
    if (time <= (*times)[0]) {
        return 0;
    }
    if (time >= times->back()){
        return times->size() - 2;
    }
    for (int i = 0; i < times->size() - 1; i++) {
        if (time >= (*times)[i] && time < (*times)[i + 1]) {
            return i;
        }   
    }
    std::cout << "Time stamp out of range." << std::endl;
    return -1;
}


Matrix4f Bone::buildLocalTransform(double time) {     
    Eigen::Vector3f posTransform = interpolatePosition(time)*1.0f;
    Eigen::Quaternionf rotateTransform = interpolateRotation(time);
    Eigen::Vector3f scaleTransform = interpolateScaling(time);

    return (Eigen::Translation3f(posTransform) * rotateTransform * Eigen::Scaling(scaleTransform)).matrix();
}

Mesh::Mesh()
  : bones(new unordered_map<string, Bone*>()),
    rootBone(nullptr)
{}

Mesh::~Mesh() {
    for (auto &p : *bones) delete p.second;
    delete bones;
}

// void Mesh::debugBones() {
//     return;
//     std::cout << "========== Bone Debug Info ==========\n";

//     for (const auto& [name, bone] : *bones) {
//         std::cout << "Bone: " << name << "\n";
//         std::cout << "  ID: " << bone->id << "\n";
//         std::cout << "  Offset Matrix:\n" << bone->offsetMatrix << "\n";
//         std::cout << "  Rest Local Transform:\n" << bone->restLocalTransformation << "\n";

//         // Position keys
//         std::cout << "  Position Keys: (" << bone->positionKeys.size() << ")\n";
//         for (size_t i = 0; i < bone->positionKeys.size(); ++i) {
//             std::cout << "    [" << i << "] t=" << bone->positionTimes[i]
//                       << " -> " << bone->positionKeys[i].transpose() << "\n";
//         }

//         // Rotation keys
//         std::cout << "  Rotation Keys: (" << bone->rotationKeys.size() << ")\n";
//         for (size_t i = 0; i < bone->rotationKeys.size(); ++i) {
//             const auto& q = bone->rotationKeys[i];
//             std::cout << "    [" << i << "] t=" << bone->rotationTimes[i]
//                       << " -> (" << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z() << ")\n";
//         }

//         // Scaling keys
//         std::cout << "  Scaling Keys: (" << bone->scalingKeys.size() << ")\n";
//         for (size_t i = 0; i < bone->scalingKeys.size(); ++i) {
//             std::cout << "    [" << i << "] t=" << bone->scalingTimes[i]
//                       << " -> " << bone->scalingKeys[i].transpose() << "\n";
//         }

//         std::cout << "  Children: " << bone->children.size() << "\n";
//         std::cout << "-------------------------------------\n";
//     }

//     std::cout << "========== End Bone Debug ==========\n";
// }

// void Mesh::debugOffsetAccuracy() {
//     // Recursively compute bind-pose global transforms
//     std::function<void(Bone*, const Matrix4f&)> dfs =
//       [&](Bone* b, const Matrix4f& parentGlobal) {
//         // 1) Compute this bone's global bind-pose
//         Matrix4f globalBind = parentGlobal * b->restLocalTransformation;

//         // 2) Multiply by your offsetMatrix
//         Matrix4f test = b->offsetMatrix * globalBind;

//         // 3) Compare to Identity
//         float err = (test - Matrix4f::Identity()).norm();
//         if (err > 1e-3f) {
//             std::cerr 
//               << "[OFFSET MISMATCH] bone " << b->name
//               << " error = " << err << "\n";
//         }

//         // Recurse
//         for (Bone* c : b->children)
//             dfs(c, globalBind);
//     };

//     if (!rootBone) return;
//     dfs(rootBone, Matrix4f::Identity());
// }

void Mesh::retrieveSceneValues(const aiScene* scene) {
    
    std::cout << "[Info] Starting scene parsing..." << std::endl;

    if (!scene || !scene->HasMeshes() || !scene->HasAnimations()) {
        std::cerr << "[Error] Scene has no meshes or animations." << std::endl;
        return;
    }

    bones = new std::unordered_map<std::string, Bone*>();
    std::unordered_map<std::string, Bone*>& boneMap = *bones;

    // Step 0: Collect all bones with weights (used by the mesh)
    std::unordered_set<std::string> usedBoneNames;
    for (unsigned int m = 0; m < scene->mNumMeshes; ++m) {
        const aiMesh* mesh = scene->mMeshes[m];
        for (unsigned int b = 0; b < mesh->mNumBones; ++b) {
            usedBoneNames.insert(mesh->mBones[b]->mName.C_Str());
        }
    }

    // Step 1: Create Bone structs from animation channels (for bones with weights)
    const aiAnimation* anim = scene->mAnimations[0];
    ticksPerSecond = anim->mTicksPerSecond;
    if (ticksPerSecond == 0.0) ticksPerSecond = 25.0;  // fallback
    duration = anim->mDuration;
    for (unsigned int i = 0; i < anim->mNumChannels; ++i) {
        const aiNodeAnim* channel = anim->mChannels[i];
        std::string name = channel->mNodeName.C_Str();

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

    // Step 3: Assign IDs deterministically
    std::vector<std::string> sortedNames;
    for (const auto& pair : boneMap)
        sortedNames.push_back(pair.first);
    std::sort(sortedNames.begin(), sortedNames.end());

    int nextId = 0;
    for (const auto& name : sortedNames) {
        boneMap[name]->id = nextId++;
    }

    // Step 4: Build hierarchy (from root node down)
    std::function<void(const aiNode*, Bone*)> buildHierarchy;
    buildHierarchy = [&](const aiNode* node, Bone* parent) {
        std::string name = node->mName.C_Str();
        Bone* current = nullptr;

        if (boneMap.count(name)) {
            current = boneMap[name];
            current->restLocalTransformation = Eigen::Map<const Matrix4f>((float*)&node->mTransformation).transpose();

            if (parent) {
                parent->children.push_back(current);
            } else {
                rootBone = current;
            }
            parent = current;
        }

        for (unsigned int i = 0; i < node->mNumChildren; ++i) {
            buildHierarchy(node->mChildren[i], parent);
        }
    };

    buildHierarchy(scene->mRootNode, nullptr);
    std::function<void(Bone*, const Eigen::Matrix4f&)> fixOffset;
    fixOffset = [&](Bone* b, const Eigen::Matrix4f& parentGlobal) {
        // 1) compute this bone's global bind-pose
        Eigen::Matrix4f globalBind = parentGlobal * b->restLocalTransformation;
        // 2) override offsetMatrix with the true inverse
        b->offsetMatrix = globalBind.inverse();
        // 3) recurse
        for (Bone* c : b->children)
            fixOffset(c, globalBind);
    };

    // run it, starting from the identity
    if (rootBone)
        fixOffset(rootBone, Eigen::Matrix4f::Identity());

    // Step 5: Fallback root if none was set
    if (!rootBone) {
        std::cerr << "[Warning] No root bone found. Using root scene node as dummy root." << std::endl;
        rootBone = new Bone();
        rootBone->name = scene->mRootNode->mName.C_Str();
        rootBone->restLocalTransformation = Eigen::Map<const Matrix4f>((float*)&scene->mRootNode->mTransformation).transpose();
        rootBone->id = nextId++;
        boneMap[rootBone->name] = rootBone;
    }

    boneMatrices.resize(boneMap.size());

    std::cout << "Total boneMatrices: " << boneMatrices.size() << std::endl;
    std::cout << "[Info] Finished building bone hierarchy and animation data." << std::endl;
}


void Bone::applyRestPose(const Matrix4f &parentTransform) {
    globalTransformation = parentTransform * restLocalTransformation;
    for (Bone* child : children) {
        child->applyRestPose(globalTransformation);
    }
}

void Mesh::animateAt(double time) {
    if (!rootBone) {
        std::cout << "No root bone" << std::endl;
        return;
    }
    double ticks = fmod(time * ticksPerSecond, duration);
    time = time * ticksPerSecond;
    Matrix4f parentTrans = Eigen::Matrix4f::Identity();

    if (ticks == 0.0) {
        rootBone->applyRestPose(parentTrans);
    } else {
        rootBone->interpolateAt(ticks, parentTrans);
    }

    boneMatrices.resize(bones->size());
    for (auto& [name, bone] : *bones) {
        boneMatrices[bone->id] = bone->globalTransformation * bone->offsetMatrix;
    }
}



Animation::Animation(const std::string &fbxPath, const GLuint shader) {
    skinProgram = shader;
    locBoneMatrices = glGetUniformLocation(skinProgram, "uBoneMatrices");
    if (locBoneMatrices == -1) std::cerr << "Could not find uBoneMatrices uniform!" << std::endl;
    locModel_ = glGetUniformLocation(shader, "uModel");

    //loading model via assimp to mesh
    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(fbxPath, aiProcess_Triangulate | aiProcess_GenSmoothNormals | aiProcess_JoinIdenticalVertices);
    if (!scene) {
        std::cerr << "FBX load error: " << importer.GetErrorString() << std::endl;
        character = nullptr;
        return;
    }
    character = new Mesh();
    character->retrieveSceneValues(scene);

    // build the VAO/VBO/EBO from character->vertices and your index list
    initMeshBuffers(scene);
}

Bone::~Bone() {}

Animation::~Animation() {
    delete character;
    glDeleteProgram(skinProgram);
    glDeleteVertexArrays(1, &VAO);
}

void Animation::initMeshBuffers(const aiScene* scene) {
    std::vector<Vertex> verts;
    std::vector<unsigned int> indices;

    Eigen::Vector3f bboxMin = Eigen::Vector3f::Constant(std::numeric_limits<float>::max());
    Eigen::Vector3f bboxMax = Eigen::Vector3f::Constant(std::numeric_limits<float>::lowest());

    // Step 1: Load vertex data (positions, normals, colors)
    for (unsigned m = 0; m < scene->mNumMeshes; ++m) {
        const aiMesh* mesh = scene->mMeshes[m];
        unsigned int vertexBase = verts.size();  // offset for indexing

        for (unsigned int v = 0; v < mesh->mNumVertices; ++v) {
            Vertex vert;

            vert.position = Eigen::Vector3f(
                mesh->mVertices[v].x,
                mesh->mVertices[v].y,
                mesh->mVertices[v].z
            );

            vert.normal = mesh->HasNormals() ?
                Eigen::Vector3f(mesh->mNormals[v].x, mesh->mNormals[v].y, mesh->mNormals[v].z) :
                Eigen::Vector3f(0, 0, 0);

            // Color (default white)
            vert.color = Eigen::Vector3f(1.0f, 1.0f, 1.0f);
            verts.push_back(vert);
        }

        for (unsigned f = 0; f < mesh->mNumFaces; ++f) {
            const aiFace& face = mesh->mFaces[f];
            if (face.mNumIndices != 3) continue;
            for (int i = 0; i < 3; ++i)
                indices.push_back(vertexBase + face.mIndices[i]);
        }
    }

    // Step 3: Accumulate bone weights per vertex
    std::vector<std::vector<std::pair<int, float>>> tempWeights(verts.size());

    unsigned int globalVertexOffset = 0;
    for (unsigned m = 0; m < scene->mNumMeshes; ++m) {
        const aiMesh* mesh = scene->mMeshes[m];

        for (unsigned int b = 0; b < mesh->mNumBones; ++b) {
            const aiBone* bone = mesh->mBones[b];
            std::string boneName = bone->mName.C_Str();
            int boneId = (*character->bones)[boneName]->id;

            for (unsigned int w = 0; w < bone->mNumWeights; ++w) {
                unsigned int vertexId = globalVertexOffset + bone->mWeights[w].mVertexId;
                float weight = bone->mWeights[w].mWeight;

                if (vertexId < tempWeights.size()) {
                    tempWeights[vertexId].push_back(std::make_pair(boneId, weight));
                }
            }
        }

        globalVertexOffset += mesh->mNumVertices;
    }

    for (size_t i = 0; i < verts.size(); ++i) {
        auto &vw = tempWeights[i];

        std::sort(vw.begin(), vw.end(),
                  [](const std::pair<int,float> &a,
                     const std::pair<int,float> &b){ return a.second > b.second; });

        size_t count = std::min(vw.size(), size_t(4));
        float  total = 0.0f;
        for (size_t j = 0; j < count; ++j) total += vw[j].second;

        for (size_t j = 0; j < 4; ++j) {
            if (j < count) {
                verts[i].boneIndices[j] = static_cast<GLuint>(vw[j].first);
                verts[i].weights[j]     = vw[j].second / total;
            } else {
                verts[i].boneIndices[j] = 0u;
                verts[i].weights[j]     = 0.0f;
            }
        }

        Eigen::Vector3f pos = verts[i].position;
        bboxMin = bboxMin.cwiseMin(pos);
        bboxMax = bboxMax.cwiseMax(pos);
    }

    character->bboxMin = bboxMin;
    character->bboxMax = bboxMax;

    // Step 4: Choose top 4 weights and normalize
    for (size_t i = 0; i < verts.size(); ++i) {
        auto& vw = tempWeights[i];
        std::sort(vw.begin(), vw.end(), [](const std::pair<int, float>& a, const std::pair<int, float>& b) {
            return a.second > b.second;
        });

        float total = 0.0f;
        size_t count = std::min(vw.size(), size_t(4));
        for (size_t j = 0; j < count; ++j)
            total += vw[j].second;

        for (size_t j = 0; j < count; ++j) {
            verts[i].boneIndices[j] = vw[j].first;
            verts[i].weights[j]     = total > 0.0f ? vw[j].second / total : 0.0f;
        }
    }

    int boneCount = character->boneMatrices.size();
    for (int i = 0; i < 5 && i < verts.size(); ++i) {
        for (int j = 0; j < 4; ++j) {
            if (verts[i].boneIndices[j] >= boneCount) {
                std::cerr << "INVALID bone index: " << verts[i].boneIndices[j] << std::endl;
            }
        }
    }

    // Step 5: Upload to GPU
    indexCount = (GLsizei)indices.size();

    GLuint VBO, EBO;
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);
    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, verts.size() * sizeof(Vertex), verts.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.data(), GL_STATIC_DRAW);

    constexpr GLsizei S = sizeof(Vertex);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, S, (void*)offsetof(Vertex, position));

    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, S, (void*)offsetof(Vertex, normal));

    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, S, (void*)offsetof(Vertex, color));

    glEnableVertexAttribArray(3);
    glVertexAttribIPointer(3, 4, GL_UNSIGNED_INT, S, (void*)offsetof(Vertex, boneIndices));

    glEnableVertexAttribArray(4);
    glVertexAttribPointer(4, 4, GL_FLOAT, GL_FALSE, S, (void*)offsetof(Vertex, weights));

    glBindVertexArray(0);

    // Step 6: Upload initial bone transforms (pose = 0)
    character->animateAt(0.0);
    const auto& boneMatrices = character->boneMatrices;
    glUseProgram(skinProgram);
    glUniformMatrix4fv(locBoneMatrices, (GLsizei)boneMatrices.size(), GL_FALSE, boneMatrices.data()->data());
    // glUseProgram(0);

    std::cout << "[Info] Buffers initialized: " << verts.size() << " vertices, " << indices.size() / 3 << " triangles.\n";

    // for (size_t i = 0; i < boneMatrices.size(); ++i) {
    //     if (!boneMatrices[i].allFinite()) {
    //         std::cerr << "[Warning] boneMatrices[" << i << "] contains NaN/Inf!" << std::endl;
    //     }
    // }
}


void Animation::animateAt(double time) {
    if (character) {
        character->animateAt(time);
    }
}

void Animation::draw() {
    if (!character) {
        std::cerr << "[Draw] No character loaded.\n";
        return;
    }

    const auto& boneMatrices = character->boneMatrices;

    glUseProgram(skinProgram);

    Eigen::Matrix4f modelMatrix = Eigen::Matrix4f::Identity();
    glUniformMatrix4fv(locModel_, 1, GL_FALSE, modelMatrix.data());


    glUniformMatrix4fv(locBoneMatrices,
                       (GLsizei)boneMatrices.size(),
                       GL_FALSE,
                       boneMatrices.data()->data());

    GLenum err = glGetError();
    if (err != GL_NO_ERROR) {
        std::cerr << "[GL ERROR] glUniformMatrix4fv failed: 0x" << std::hex << err << std::dec << "\n";
    }

    glBindVertexArray(VAO);
    err = glGetError();
    if (err != GL_NO_ERROR) {
        std::cerr << "[GL ERROR] glBindVertexArray failed: 0x" << std::hex << err << std::dec << "\n";
    }

    glDrawElements(GL_TRIANGLES, indexCount, GL_UNSIGNED_INT, nullptr);
    err = glGetError();
    if (err != GL_NO_ERROR) {
        std::cerr << "[GL ERROR] glDrawElements failed: 0x" << std::hex << err << std::dec << "\n";
    }
}
