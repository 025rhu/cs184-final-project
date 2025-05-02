#include "animator.h"
#include "Eigen/src/Geometry/Quaternion.h"
// #include "GLFW/glfw3.h"
#include "nanogui/common.h"
#include <cfloat>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>
// #include "shader.h"
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
    Matrix4f globalTransform = parentTransform * localTransform;    // try parent * local if this looks wonky
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
//     int i = findIndex(time, &rotationTimes);
// if (i < 0 || i + 1 >= rotationKeys.size()) {
//     std::cerr << "[interpolateRotation] Invalid index at t=" << time << std::endl;
//     return Eigen::Quaternionf::Identity();
// }

// double t0 = rotationTimes[i];
// double t1 = rotationTimes[i + 1];
// Eigen::Quaternionf q0 = rotationKeys[i];
// Eigen::Quaternionf q1 = rotationKeys[i + 1];

// std::cout << "[interpolateRotation] t = " << time
//           << ", t0 = " << t0 << ", t1 = " << t1
//           << ", q0 = (" << q0.w() << "," << q0.x() << "," << q0.y() << "," << q0.z() << ")"
//           << ", q1 = (" << q1.w() << "," << q1.x() << "," << q1.y() << "," << q1.z() << ")"
//           << std::endl;

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
    Eigen::Vector3f posTransform = interpolatePosition(time);
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
    // delete vertices;
}

void Mesh::debugBones() {
    std::cout << "========== Bone Debug Info ==========\n";

    for (const auto& [name, bone] : *bones) {
        std::cout << "Bone: " << name << "\n";
        std::cout << "  ID: " << bone->id << "\n";
        std::cout << "  Offset Matrix:\n" << bone->offsetMatrix << "\n";
        std::cout << "  Rest Local Transform:\n" << bone->restLocalTransformation << "\n";

        // Position keys
        std::cout << "  Position Keys: (" << bone->positionKeys.size() << ")\n";
        for (size_t i = 0; i < bone->positionKeys.size(); ++i) {
            std::cout << "    [" << i << "] t=" << bone->positionTimes[i]
                      << " -> " << bone->positionKeys[i].transpose() << "\n";
        }

        // Rotation keys
        std::cout << "  Rotation Keys: (" << bone->rotationKeys.size() << ")\n";
        for (size_t i = 0; i < bone->rotationKeys.size(); ++i) {
            const auto& q = bone->rotationKeys[i];
            std::cout << "    [" << i << "] t=" << bone->rotationTimes[i]
                      << " -> (" << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z() << ")\n";
        }

        // Scaling keys
        std::cout << "  Scaling Keys: (" << bone->scalingKeys.size() << ")\n";
        for (size_t i = 0; i < bone->scalingKeys.size(); ++i) {
            std::cout << "    [" << i << "] t=" << bone->scalingTimes[i]
                      << " -> " << bone->scalingKeys[i].transpose() << "\n";
        }

        std::cout << "  Children: " << bone->children.size() << "\n";
        std::cout << "-------------------------------------\n";
    }

    std::cout << "========== End Bone Debug ==========\n";
}


// CORRECT
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

    // Step 2: Assign offset matrices to bones from mesh data
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

    debugBones();
}



// potential buggy point if bone matrices are not in the same order
// void Mesh::getBoneMatrices(Bone* bone, vector<Eigen::Matrix4f>& boneMatrices) {
//     // retreives the bone matrices recursively and 
//     boneMatrices.push_back(bone->globalTransformation * bone->offsetMatrix);
//     for (auto *c : bone->children) {
//         getBoneMatrices(c, boneMatrices);
//     }
// }

// void Mesh::getBoneMatrices() {
//     // cout << "in get bone matrices" << endl;
//     for (auto& pair : *bones) {
//         boneMatrices[pair.second->id] = pair.second->globalTransformation * pair.second->offsetMatrix;
//     }
    // cout << "finished get bone matrices" << endl;
// }


void Mesh::animateAt(double time) {

    if (!rootBone) {
        std::cout << "No root bone" << std::endl;
        return;
    } 
    Matrix4f parentTrans = Eigen::Matrix4f::Identity();
    rootBone->interpolateAt(time, parentTrans);
    // boneMatrices.clear();
    // getBoneMatrices();
    boneMatrices.resize(bones->size());
    for (auto& [name, bone] : *bones) {
        boneMatrices[bone->id] = bone->globalTransformation * bone->offsetMatrix;
        // boneMatrices[bone->id] *= 100.0f;
    }

    // for (size_t i = 0; i < boneMatrices.size(); ++i) {
    //     std::cout << "Bone " << i << " matrix:\n" << boneMatrices[i] << "\n";
    // }

}







Animation::Animation(const std::string &fbxPath, const GLuint shader) {
    skinProgram = shader;
    locBoneMatrices = glGetUniformLocation(skinProgram, "uBoneMatrices");
    if (locBoneMatrices == -1) std::cerr << "Could not find uBoneMatrices uniform!" << std::endl;
    locModel_ = glGetUniformLocation(shader, "uModel");




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
    std::vector<Vertex> verts;
    std::vector<unsigned int> indices;

    // Step 1: Load vertex data (positions, normals, colors)
    for (unsigned m = 0; m < scene->mNumMeshes; ++m) {
        const aiMesh* mesh = scene->mMeshes[m];
        unsigned int vertexBase = verts.size();  // offset for indexing

        for (unsigned int v = 0; v < mesh->mNumVertices; ++v) {
            Vertex vert;

            // Position
            vert.position = Eigen::Vector3f(
                mesh->mVertices[v].x,
                mesh->mVertices[v].y,
                mesh->mVertices[v].z
            );

            // Normal
            vert.normal = mesh->HasNormals() ?
                Eigen::Vector3f(mesh->mNormals[v].x, mesh->mNormals[v].y, mesh->mNormals[v].z) :
                Eigen::Vector3f(0, 0, 0);

            // Color (default white)
            vert.color = Eigen::Vector3f(1.0f, 1.0f, 1.0f);

            // Initialize bone data
        
            // vert.boneIndices.resize(4, 0);
            // vert.weights.resize(4, 0.0f);

            verts.push_back(vert);
        }

        // Step 2: Load indices
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
    cout << "[DEBUG DEBUG DEBUG] for loop for bone weights" << endl;
    for (unsigned m = 0; m < scene->mNumMeshes; ++m) {
        const aiMesh* mesh = scene->mMeshes[m];

        for (unsigned int b = 0; b < mesh->mNumBones; ++b) {
            const aiBone* bone = mesh->mBones[b];
            std::string boneName = bone->mName.C_Str();
            int boneId = (*character->bones)[boneName]->id;
            cout << "Bone Name: " << boneName << ", Bone ID: " << boneId << endl;

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
    }


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


    for (int i = 0; i < std::min(10, (int)verts.size()); ++i) {
        std::cout << "Vertex " << i << " boneIndices: ";
        for (int j = 0; j < 4; ++j)
            std::cout << verts[i].boneIndices[j] << " ";
        std::cout << " | weights: ";
        for (int j = 0; j < 4; ++j)
            std::cout << verts[i].weights[j] << " ";
        std::cout << "\n";
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
    glUseProgram(0);

    std::cout << "[Info] Buffers initialized: " << verts.size() << " vertices, " << indices.size() / 3 << " triangles.\n";

    for (size_t i = 0; i < boneMatrices.size(); ++i) {
        if (!boneMatrices[i].allFinite()) {
            std::cerr << "[Warning] boneMatrices[" << i << "] contains NaN/Inf!" << std::endl;
        }
    }
}



//----------------------------------------------------------------------
// Advance your mesh’s bone matrices
//----------------------------------------------------------------------
void Animation::animateAt(double time) {
    if (character) {
        character->animateAt(time);
    }
    for (int i = 0; i < character->boneMatrices.size(); ++i) {
        Eigen::Matrix3f m = character->boneMatrices[i].block<3,3>(0,0);
        float scale = m.norm() / sqrt(3.f);
        std::cout << "BoneMatrix " << i << " approx scale: " << scale << std::endl;
    }
    
}

void Animation::draw() {
    if (!character) {
        std::cerr << "[Draw] No character loaded.\n";
        return;
    }

    const auto& boneMatrices = character->boneMatrices;

    glUseProgram(skinProgram);

    // Eigen::Matrix4f modelMatrix = Eigen::Matrix4f::Identity();
    // glUniformMatrix4fv(locModel_, 1, GL_FALSE, modelMatrix.data());
    Eigen::Matrix4f modelMatrix = Eigen::Matrix4f::Identity();
    //modelMatrix.block<3,3>(0,0) *= 100.0f;  // scale uniformly up
    glUniformMatrix4fv(locModel_, 1, GL_FALSE, modelMatrix.data());


    // DEBUG: Check matrix count
    // std::cout << "[Draw] Uploading " << boneMatrices.size() << " bone matrices.\n";

    // DEBUG: Check for NaNs in first few matrices
    for (size_t i = 0; i < std::min<size_t>(boneMatrices.size(), 5); ++i) {
        if (!boneMatrices[i].allFinite()) {
            std::cerr << "[Warning] boneMatrices[" << i << "] has NaN or Inf\n";
        } else {
            std::cout << "[BoneMatrix " << i << "] Sample value (0,3): " << boneMatrices[i](0, 3) << "\n";
        }

        std::cout << "[BoneMatrix " << i << "] trace: " << boneMatrices[i].trace()
          << " norm: " << boneMatrices[i].block<3,3>(0,0).norm() << "\n";

    }

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

    // std::cout << "[Draw] Drawing " << indexCount / 3 << " triangles.\n";

    glDrawElements(GL_TRIANGLES, indexCount, GL_UNSIGNED_INT, nullptr);
    err = glGetError();
    if (err != GL_NO_ERROR) {
        std::cerr << "[GL ERROR] glDrawElements failed: 0x" << std::hex << err << std::dec << "\n";
    }
}
