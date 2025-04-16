// #include <nanogui/screen.h>
// #include <nanogui/window.h>
// #include <nanogui/opengl.h>

// // #include <Eigen/Core>
// // #include <Eigen/Dense>

// #include <GLFW/glfw3.h>
// #include <iostream>
// #include <cmath>

// #include "CGL/matrix4x4.h"
// #include "shader.h"
// #include "model.h"
// #include "CGL/CGL.h"



// // Perspective matrix builder
// CGL::Matrix4x4 makePerspective(float fovy, float aspect, float zNear, float zFar) {
//     float tanHalfFovy = std::tan(fovy / 2.0f);

//     CGL::Matrix4x4 mat;
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

//         Viewer() : nanogui::Screen(Eigen::Vector2i(1024, 768), "FBX Viewer", true) {
//             // Load and compile shaders
//             shader = createShaderProgram("shaders/Default.vert", "shaders/Default.frag");

//             // OpenGL config
//             glEnable(GL_DEPTH_TEST);

//             // Load your model (relative path if run from build/)
//             loadModel("../models/donut.fbx");
//         }

//         virtual void drawContents() override {
//             glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
//             glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

//             glUseProgram(shader);

//             // Matrices
//             CGL::Matrix4x4 model = CGL::Matrix4x4::identity();
//             CGL::Matrix4x4 view = CGL::Matrix4x4::identity();
//             view(2, 3) = -150.0f;
//             float aspect = (float)mSize.x() / (float)mSize.y();
//             CGL::Matrix4x4 proj = makePerspective(45.0f * M_PI / 180.0f, aspect, 0.1f, 100.0f);

//             // Upload uniforms
//             GLint locModel = glGetUniformLocation(shader, "uM");
//             GLint locView  = glGetUniformLocation(shader, "uV");
//             GLint locProj  = glGetUniformLocation(shader, "uP");

//             // glUniformMatrix4fv(locModel, 1, GL_FALSE, model);
//             // glUniformMatrix4fv(locView,  1, GL_FALSE, view.data());
//             // glUniformMatrix4fv(locProj,  1, GL_FALSE, proj.data());

//             // Draw mesh
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

#include <nanogui/screen.h>
#include <nanogui/window.h>
#include <nanogui/opengl.h>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <GLFW/glfw3.h>
#include <iostream>
#include <cmath>

#include "shader.h"
#include "model.h"

// Perspective matrix builder
Eigen::Matrix4f makePerspective(float fovy, float aspect, float zNear, float zFar) {
    float tanHalfFovy = std::tan(fovy / 2.0f);

    Eigen::Matrix4f mat = Eigen::Matrix4f::Zero();
    mat(0, 0) = 1.0f / (aspect * tanHalfFovy);
    mat(1, 1) = 1.0f / (tanHalfFovy);
    mat(2, 2) = -(zFar + zNear) / (zFar - zNear);
    mat(2, 3) = -2.0f * zFar * zNear / (zFar - zNear);
    mat(3, 2) = -1.0f;
    return mat;
}

int main() {
    nanogui::init();

    class Viewer : public nanogui::Screen {
    public:
        GLuint shader;

        Viewer() : nanogui::Screen(Eigen::Vector2i(1024, 768), "FBX Viewer", true) {
            // Load and compile shaders
            shader = createShaderProgram("shaders/Default.vert", "shaders/Default.frag");

            // OpenGL config
            glEnable(GL_DEPTH_TEST);
            //Mode(GL_FRONT_AND_BACK, GL_LINE);

            // Load your model (relative path if run from build/)
            loadModel("../models/bear.fbx");
        }

        Eigen::Matrix4f lookAt(const Eigen::Vector3f& eye,
            const Eigen::Vector3f& center,
            const Eigen::Vector3f& up)
        {
        Eigen::Vector3f f = (center - eye).normalized();
        Eigen::Vector3f s = f.cross(up).normalized();
        Eigen::Vector3f u = s.cross(f);

        Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
        m.block<1,3>(0,0) =  s.transpose();
        m.block<1,3>(1,0) =  u.transpose();
        m.block<1,3>(2,0) = -f.transpose();
        m(0,3) = -s.dot(eye);
        m(1,3) = -u.dot(eye);
        m(2,3) =  f.dot(eye);
        return m;
        }

        virtual void drawContents() override {
            glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            glUseProgram(shader);

            // Matrices
            Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
            // Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
            Eigen::Matrix4f view =
            lookAt(/*eye   */ { 0.0f, 10.0f, 0.0f },   //  ➜ from +Y
                /*center*/ { 0.0f, -1.4f, 0.0f },
                /*up    */ { 0.0f, 0.0f, 1.0f });
            // view(2, 3) = -175.0f;
            //view(2, 3) = -5.0f;
            model(2, 3) = -1.4f;
            float aspect = (float)mSize.x() / (float)mSize.y();
            Eigen::Matrix4f proj = makePerspective(45.0f * M_PI / 180.0f, aspect, 0.1f, 100.0f);

            // Upload uniforms
            GLint locModel = glGetUniformLocation(shader, "uM");
            GLint locView  = glGetUniformLocation(shader, "uV");
            GLint locProj  = glGetUniformLocation(shader, "uP");

            glUniformMatrix4fv(locModel, 1, GL_FALSE, model.data());
            glUniformMatrix4fv(locView,  1, GL_FALSE, view.data());
            glUniformMatrix4fv(locProj,  1, GL_FALSE, proj.data());

            // Draw mesh
            glBindVertexArray(VAO);
            glDrawElements(GL_TRIANGLES, indexCount, GL_UNSIGNED_INT, 0);
            glBindVertexArray(0);
        }
    };

    Viewer* screen = new Viewer();
    screen->setVisible(true);
    screen->drawAll();

    nanogui::mainloop();
    nanogui::shutdown();
    return 0;
}