// // #include <iostream>

// // int main() {
// //     std::cout << "Hello, World!" << std::endl;
// //     return 0;
// // }

// #include <assimp/Importer.hpp>
// #include <assimp/scene.h>
// #include <assimp/postprocess.h>

// #include <vector>
// #include <GL/glew.h> // Or glad if you're using that
// #include <glm/glm.hpp>
// #include <glm/gtc/type_ptr.hpp>

// GLuint vao, vbo, nbo, ebo; // Vertex Array Object, Buffers

// void loadModel(const std::string& path) {
//     Assimp::Importer importer;

//     const aiScene* scene = importer.ReadFile(path,
//         aiProcess_Triangulate |
//         aiProcess_GenSmoothNormals |
//         aiProcess_JoinIdenticalVertices |
//         aiProcess_ImproveCacheLocality);

//     if (!scene || !scene->HasMeshes()) {
//         std::cerr << "Assimp failed: " << importer.GetErrorString() << std::endl;
//         return;
//     }

//     const aiMesh* mesh = scene->mMeshes[0]; // Load first mesh

//     std::vector<float> vertices;
//     std::vector<float> normals;
//     std::vector<unsigned int> indices;

//     // Load vertex positions
//     for (unsigned int i = 0; i < mesh->mNumVertices; ++i) {
//         aiVector3D pos = mesh->mVertices[i];
//         vertices.push_back(pos.x);
//         vertices.push_back(pos.y);
//         vertices.push_back(pos.z);

//         aiVector3D norm = mesh->mNormals[i];
//         normals.push_back(norm.x);
//         normals.push_back(norm.y);
//         normals.push_back(norm.z);
//     }

//     // Load indices
//     for (unsigned int i = 0; i < mesh->mNumFaces; ++i) {
//         const aiFace& face = mesh->mFaces[i];
//         for (unsigned int j = 0; j < face.mNumIndices; ++j) {
//             indices.push_back(face.mIndices[j]);
//         }
//     }

//     // Upload to OpenGL
//     glGenVertexArrays(1, &vao);
//     glBindVertexArray(vao);

//     // Vertex positions
//     glGenBuffers(1, &vbo);
//     glBindBuffer(GL_ARRAY_BUFFER, vbo);
//     glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);
//     glEnableVertexAttribArray(0); // location 0 for aVertex
//     glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);

//     // Normals
//     glGenBuffers(1, &nbo);
//     glBindBuffer(GL_ARRAY_BUFFER, nbo);
//     glBufferData(GL_ARRAY_BUFFER, normals.size() * sizeof(float), normals.data(), GL_STATIC_DRAW);
//     glEnableVertexAttribArray(1); // location 1 for aNormal
//     glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);

//     // Indices
//     glGenBuffers(1, &ebo);
//     glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
//     glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.data(), GL_STATIC_DRAW);

//     glBindVertexArray(0); // Unbind VAO
// }

// main.cpp

#include <nanogui/screen.h>
#include <nanogui/window.h>
#include <nanogui/layout.h>
#include <nanogui/label.h>

using namespace nanogui;

int main(int /* argc */, char ** /* argv */) {
    nanogui::init();

    {
        // Create a screen (i.e. window) of size 800x600
        Screen *screen = new Screen(Vector2i(800, 600), "NanoGUI Test Window");

        // Create a window widget inside the screen
        Window *win = new Window(screen, "Hello, NanoGUI!");
        win->setPosition(Vector2i(15, 15));
        win->setLayout(new GroupLayout());

        // Add a label
        new Label(win, "Window successfully created!", "sans-bold");

        screen->setVisible(true);
        screen->performLayout();

        // Main GUI loop
        nanogui::mainloop();
    }

    nanogui::shutdown();
    return 0;
}

