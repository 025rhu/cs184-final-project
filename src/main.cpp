#include <nanogui/screen.h>
#include <nanogui/window.h>
#include <nanogui/opengl.h>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <GLFW/glfw3.h>
#include <iostream>
#include <cmath>

#include "nanogui/common.h"
#include "shader.h"
#include "animator.h"


static Eigen::Matrix4f makePerspective(float fovy, float aspect, float zNear, float zFar) {
    float tanHalfFovy = std::tan(fovy / 2.0f);

    Eigen::Matrix4f mat = Eigen::Matrix4f::Zero();
    mat(0, 0) = 1.0f / (aspect * tanHalfFovy);
    mat(1, 1) = 1.0f / (tanHalfFovy);
    mat(2, 2) = -(zFar + zNear) / (zFar - zNear);
    mat(2, 3) = -2.0f * zFar * zNear / (zFar - zNear);
    mat(3, 2) = -1.0f;
    return mat;
}

class Viewer : public nanogui::Screen {
    public:
        
        Animation* animation;
        GLuint shader;

        Viewer() : nanogui::Screen(Eigen::Vector2i(1024, 768), "Viewer", true) {
            shader = createShaderProgram("shaders/skinning.vert", "shaders/skinning.frag");
            glUseProgram(shader);

            // access uniform variables declared in shader program that we loaded in
            locModel_ = glGetUniformLocation(shader, "uModel");
            locView_  = glGetUniformLocation(shader, "uView");
            locProj_  = glGetUniformLocation(shader, "uProjection");

            // viewMatrix_ = lookAt({0, 10, 0}, {0, 0, 0}, {0, 0, 1});
            // viewMatrix_ = lookAt({-5, 0, 0}, {0, 0, 0}, {0, 1, 0});
            viewMatrix_ = lookAt({0, 12, 1}, {0, 0, 1}, {0, 0, 1});


           
            // viewMatrix_ = lookAt({5, 5, 5}, {0, 0, 0}, {0, 1, 0});





            float aspect = float(mSize.x())/float(mSize.y());
            // projMatrix_ = makePerspective(45.0f * M_PI / 180.0f, aspect, 0.1f, 100.0f);
            projMatrix_ = makePerspective(30.0f * M_PI / 180.0f, aspect, 0.1f, 100.0f);


            glUniformMatrix4fv(locView_,  1, GL_FALSE, viewMatrix_.data());
            glUniformMatrix4fv(locProj_,  1, GL_FALSE, projMatrix_.data());

            glEnable(GL_DEPTH_TEST);
            //glPolygonMode(GL_FRONT_AND_BACK, GL_LINE); // to enable mesh mode

            int width, height;
            glfwGetFramebufferSize(glfwWindow(), &width, &height);
            glViewport(0, 0, width, height);
        }

        // custom impl of a "look-at" view matrix, equivalent to glm::lookAt()
        // returns a matrix that transforms world-space points into camera/view space
        Eigen::Matrix4f lookAt(const Eigen::Vector3f& eye, const Eigen::Vector3f& center, const Eigen::Vector3f& up) {
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

        void drawContents() override {
            if (animation == NULL || animation->character == NULL)
                return;
            
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            glUseProgram(shader);

            // Advance animation time
            double now = glfwGetTime();
            if (animation->startTime < 0.0)
                animation->startTime = now;
        
            double t = now - animation->startTime;

            animation->character->animateAt(t);
            animation->draw();
        }

        void setCameraFromBoundingBox(const Eigen::Vector3f& min, const Eigen::Vector3f& max) {
            Eigen::Vector3f center = 0.5f * (min + max);
            Eigen::Vector3f size   = max - min;
            float radius           = size.norm() * 0.5f;
            float fovY             = 45.0f * M_PI / 180.0f;
            float distance         = radius / std::tan(fovY / 2.0f);
    
            Eigen::Vector3f viewDir = Eigen::Vector3f(-1, 1, -1);
            Eigen::Vector3f eye    = center + distance * viewDir;
            Eigen::Vector3f up = Eigen::Vector3f(0, 1, 0);
            viewMatrix_ = lookAt({0, 5, 3}, {0, 0, 0}, {0, 3, 1});
            // viewMatrix_ = lookAt(eye, center, up);
    
            glUseProgram(shader);
            glUniformMatrix4fv(locView_, 1, GL_FALSE, viewMatrix_.data());
        }

    private:
        GLint locModel_, locView_, locProj_;
        Eigen::Matrix4f viewMatrix_, projMatrix_;
};

int main() {
    nanogui::init();
    Viewer* screen = new Viewer();
    // std::cout << "initialized viewer." << std::endl;

    Animation* anim = new Animation("../models/bear_one_mesh.fbx", screen->shader);
    // std::cout << "initialized animation." << std::endl;

    screen->animation = anim;
    Eigen::Vector3f min = anim->character->bboxMin;
    Eigen::Vector3f max = anim->character->bboxMax;
    // screen->setCameraFromBoundingBox(min, max);
    screen->setVisible(true);
    screen->drawAll();
    nanogui::mainloop();
    nanogui::shutdown();
}