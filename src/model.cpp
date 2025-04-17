#include "model.h"
#include <vector>
#include <iostream>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <Eigen/Core>
#include <Eigen/Dense>


MeshFrame loadModel(const std::string& path,
                    Eigen::Vector3f*    outCenter,
                    float*              outHeight) {

    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(path, aiProcess_Triangulate | aiProcess_GenNormals | aiProcess_JoinIdenticalVertices);

    if (!scene || !scene->HasMeshes()) {
        std::cerr << "Failed to load model: " << path << std::endl;
        return {};
    }

    std::vector<float> vertices, normals, colors;
    std::vector<unsigned int> indices;

    aiVector3D min(1e10f, 1e10f, 1e10f);
    aiVector3D max(-1e10f, -1e10f, -1e10f);

    unsigned int vertexOffset = 0;

    for (unsigned int m = 0; m < scene->mNumMeshes; ++m) {
        const aiMesh* mesh = scene->mMeshes[m];

        bool hasColors = mesh->HasVertexColors(0);

        for (unsigned int i = 0; i < mesh->mNumVertices; ++i) {
            aiVector3D pos = mesh->mVertices[i];
            aiVector3D norm = mesh->mNormals[i];

            min.x = std::min(min.x, pos.x);
            min.y = std::min(min.y, pos.y);
            min.z = std::min(min.z, pos.z);
            max.x = std::max(max.x, pos.x);
            max.y = std::max(max.y, pos.y);
            max.z = std::max(max.z, pos.z);

            vertices.push_back(pos.x);
            vertices.push_back(pos.y);
            vertices.push_back(pos.z);

            normals.push_back(norm.x);
            normals.push_back(norm.y);
            normals.push_back(norm.z);

            if (hasColors) {
                aiColor4D color = mesh->mColors[0][i];
                colors.push_back(color.r);
                colors.push_back(color.g);
                colors.push_back(color.b);
            } else {
                // Default white if no vertex colors
                colors.push_back(1.0f);
                colors.push_back(1.0f);
                colors.push_back(1.0f);
            }
        }

        for (unsigned int i = 0; i < mesh->mNumFaces; ++i) {
            const aiFace& face = mesh->mFaces[i];
            for (unsigned int j = 0; j < face.mNumIndices; ++j) {
                indices.push_back(face.mIndices[j] + vertexOffset);
            }
        }

        vertexOffset += mesh->mNumVertices;
    }

    int indexCount = indices.size();

    aiVector3D center = (min + max) * 0.5f;
    Eigen::Vector3f modelCenter = Eigen::Vector3f(center.x, center.y, center.z);
    float modelHeight = max.y - min.y;

    if (outCenter) *outCenter = {center.x, center.y, center.z};
    if (outHeight) *outHeight = max.y - min.y;

    std::cout << "Model has " << indexCount / 3 << " triangles across " << scene->mNumMeshes << " mesh(es)" << std::endl;
    std::cout << "Model center: (" << center.x << ", " << center.y << ", " << center.z << ")\n";
    std::cout << "Model height: " << modelHeight << std::endl;

    // now local per fbx file
    GLuint VAO, VBO, NBO, CBO, EBO;
    // Upload data to GPU
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &NBO);
    glGenBuffers(1, &EBO);
    glGenBuffers(1, &CBO);

    glBindVertexArray(VAO);

    // Positions
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(0);

    // Normals
    glBindBuffer(GL_ARRAY_BUFFER, NBO);
    glBufferData(GL_ARRAY_BUFFER, normals.size() * sizeof(float), normals.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(1);

    // Colors
    glBindBuffer(GL_ARRAY_BUFFER, CBO);
    glBufferData(GL_ARRAY_BUFFER, colors.size() * sizeof(float), colors.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(2);

    // Indices
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.data(), GL_STATIC_DRAW);

    glBindVertexArray(0);

    
    
    // return the MeshFrame
    MeshFrame ret;
    ret.indexCount = static_cast<GLuint>(indices.size());
    ret.vao = VAO;
    return ret;
}
