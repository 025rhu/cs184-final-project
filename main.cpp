// #include <nanogui/screen.h>
// #include <nanogui/window.h>
// #include <nanogui/opengl.h>

// #include <assimp/Importer.hpp>
// #include <assimp/scene.h>
// #include <assimp/postprocess.h>

// #include <Eigen/Core>
// #include <Eigen/Dense>

// #include <iostream>
// #include <fstream>
// #include <sstream>
// #include <vector>
// #include <string>
// #include <cmath> // for M_PI

// // Load shader from file
// std::string loadShaderSource(const std::string& path) {
//     std::ifstream file(path);
//     std::stringstream buffer;
//     buffer << file.rdbuf();
//     return buffer.str();
// }

// // Compile shader
// GLuint compileShader(GLenum type, const std::string& src) {
//     GLuint shader = glCreateShader(type);
//     const char* cstr = src.c_str();
//     glShaderSource(shader, 1, &cstr, nullptr);
//     glCompileShader(shader);

//     GLint success;
//     glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
//     if (!success) {
//         char info[512];
//         glGetShaderInfoLog(shader, 512, nullptr, info);
//         std::cerr << "Shader compile error:\n" << info << std::endl;
//     }

//     return shader;
// }

// // Link shaders into a program
// GLuint createShaderProgram(const std::string& vertPath, const std::string& fragPath) {
//     std::string vertCode = loadShaderSource(vertPath);
//     std::string fragCode = loadShaderSource(fragPath);

//     GLuint vertShader = compileShader(GL_VERTEX_SHADER, vertCode);
//     GLuint fragShader = compileShader(GL_FRAGMENT_SHADER, fragCode);

//     GLuint program = glCreateProgram();
//     glAttachShader(program, vertShader);
//     glAttachShader(program, fragShader);
//     glLinkProgram(program);

//     glDeleteShader(vertShader);
//     glDeleteShader(fragShader);

//     return program;
// }

// // Globals
// GLuint VAO, VBO, NBO, EBO;
// int indexCount = 0;

// void loadModel(const std::string& path) {
//     Assimp::Importer importer;
//     const aiScene* scene = importer.ReadFile(path, aiProcess_Triangulate | aiProcess_GenNormals);
//     if (!scene || !scene->HasMeshes()) {
//         std::cerr << "Failed to load model: " << path << std::endl;
//         return;
//     }

//     const aiMesh* mesh = scene->mMeshes[0];

//     std::vector<float> vertices, normals;
//     std::vector<unsigned int> indices;

//     for (unsigned int i = 0; i < mesh->mNumVertices; ++i) {
//         aiVector3D pos = mesh->mVertices[i];
//         aiVector3D norm = mesh->mNormals[i];

//         vertices.push_back(pos.x);
//         vertices.push_back(pos.y);
//         vertices.push_back(pos.z);

//         normals.push_back(norm.x);
//         normals.push_back(norm.y);
//         normals.push_back(norm.z);
//     }

//     for (unsigned int i = 0; i < mesh->mNumFaces; ++i) {
//         const aiFace& face = mesh->mFaces[i];
//         for (unsigned int j = 0; j < face.mNumIndices; ++j) {
//             indices.push_back(face.mIndices[j]);
//         }
//     }

//     indexCount = indices.size();

//     glGenVertexArrays(1, &VAO);
//     glGenBuffers(1, &VBO);
//     glGenBuffers(1, &NBO);
//     glGenBuffers(1, &EBO);

//     glBindVertexArray(VAO);

//     glBindBuffer(GL_ARRAY_BUFFER, VBO);
//     glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);
//     glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
//     glEnableVertexAttribArray(0);

//     glBindBuffer(GL_ARRAY_BUFFER, NBO);
//     glBufferData(GL_ARRAY_BUFFER, normals.size() * sizeof(float), normals.data(), GL_STATIC_DRAW);
//     glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);
//     glEnableVertexAttribArray(1);

//     glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
//     glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.data(), GL_STATIC_DRAW);

//     glBindVertexArray(0);
// }

// // Perspective matrix (manual since Eigen has no built-in perspective)
// Eigen::Matrix4f makePerspective(float fovy, float aspect, float zNear, float zFar) {
//     float tanHalfFovy = std::tan(fovy / 2.0f);

//     Eigen::Matrix4f mat = Eigen::Matrix4f::Zero();
//     mat(0, 0) = 1.0f / (aspect * tanHalfFovy);
//     mat(1, 1) = 1.0f / (tanHalfFovy);
//     mat(2, 2) = -(zFar + zNear) / (zFar - zNear);
//     mat(2, 3) = -2.0f * zFar * zNear / (zFar - zNear);
//     mat(3, 2) = -1.0f;
//     return mat;
// }

// int main() {
//     nanogui::init();

//     class Viewer : public nanogui::Screen {
//     public:
//         GLuint shader;

//         Viewer()
//             : nanogui::Screen(Eigen::Vector2i(1024, 768), "Assimp FBX Viewer", true) {
//             shader = createShaderProgram("shaders/vertex.glsl", "shaders/fragment.glsl");
//             glEnable(GL_DEPTH_TEST);
//             loadModel("model.fbx");
//         }

//         virtual void drawContents() override {
//             glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
//             glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

//             glUseProgram(shader);

//             Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
//             Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
//             view.block<3, 1>(0, 3) = Eigen::Vector3f(0, 0, -3);
//             float aspect = 1024.f / 768.f;
//             Eigen::Matrix4f proj = makePerspective(45.0f * M_PI / 180.0f, aspect, 0.1f, 100.0f);

//             GLint locModel = glGetUniformLocation(shader, "uM");
//             GLint locView  = glGetUniformLocation(shader, "uV");
//             GLint locProj  = glGetUniformLocation(shader, "uP");

//             glUniformMatrix4fv(locModel, 1, GL_FALSE, model.data());
//             glUniformMatrix4fv(locView,  1, GL_FALSE, view.data());
//             glUniformMatrix4fv(locProj,  1, GL_FALSE, proj.data());

//             glBindVertexArray(VAO);
//             glDrawElements(GL_TRIANGLES, indexCount, GL_UNSIGNED_INT, 0);
//             glBindVertexArray(0);
//         }
//     };

//     Viewer* screen = new Viewer();
//     screen->setVisible(true);
//     screen->drawAll();

//     nanogui::mainloop();
//     nanogui::shutdown();
//     return 0;
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
