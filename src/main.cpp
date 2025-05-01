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
#include "model.h"
#include "animatedObject.h"
#include "animator.h"


// NEW FRAMEWORK YALL!
// MAIN WILL ONLY SET UP THE SCREEN
// ANIMATION WILL SET UP THE PROPER CAMERAS, PERSPECTIVES, ETC.

// class Viewer : public nanogui::Screen {
//     public:
//         Viewer() : nanogui::Screen(Eigen::Vector2i(1024, 768), "My App", true) {
//             // init stuff
//         }
    
//         virtual void drawContents() override {
//             // your custom draw code
//             animation.animate();
//             animation.draw();
//         }
    
//     private:
//         Animation animation;
//     };

class Viewer : public nanogui::Screen {
    public:
    Viewer(Animation* animation)
            : nanogui::Screen(Eigen::Vector2i(1024, 768), "Viewer", true),
                animation(animation) {}

        void drawContents() override {
            double now = glfwGetTime();
            if (animation->startTime < 0.0)
                animation->startTime = now;

            double t = now - animation->startTime;
            animation->character->animateAt(t);
            animation->draw(&animation->character->boneMatrices);
        }

    private:
        Animation* animation;
};
    


int main() {
    // nanogui::init();
    // // Create a Screen that won’t do any drawing by itself:
    // nanogui::Screen *screen =
    //   new nanogui::Screen({1024,768}, "FBX Animation Viewer", true);

    // // Give that Screen to your Animation, which will install its own draw callback
    // Animation *anim = new Animation(screen);

    // screen->setVisible(true);
    // screen->drawAll();    // first frame
    // nanogui::mainloop();  // drives your callback every frame
    // nanogui::shutdown();
    // return 0;
    nanogui::init();
    Animation* anim = new Animation(); // or however you construct it
    Viewer* screen = new Viewer(anim);
    screen->setVisible(true);
    screen->drawAll();
    nanogui::mainloop();
}


// Perspective matrix builder
// static Eigen::Matrix4f makePerspective(float fovy, float aspect, float zNear, float zFar) {
//     float tanHalfFovy = std::tan(fovy / 2.0f);

//     Eigen::Matrix4f mat = Eigen::Matrix4f::Zero();
//     mat(0, 0) = 1.0f / (aspect * tanHalfFovy);
//     mat(1, 1) = 1.0f / (tanHalfFovy);
//     mat(2, 2) = -(zFar + zNear) / (zFar - zNear);
//     mat(2, 3) = -2.0f * zFar * zNear / (zFar - zNear);
//     mat(3, 2) = -1.0f;
//     return mat;
// }

// // class inherits nanogui::Screen
// class Viewer : public nanogui::Screen {
//     public:
//         Viewer() : nanogui::Screen(Eigen::Vector2i(1024, 768), "FBX Viewer", true) {
//             // Load and compile shaders
//             shader = createShaderProgram("shaders/Default.vert", "shaders/Default.frag");

//             // OpenGL config
//             glEnable(GL_DEPTH_TEST);
//             anim.secondsPerFrame = 0.5f;          // 10 fps
//             bool ok = anim.load({
//             "../models/bear.fbx",
//             "../models/bear_frame0.fbx",
//             "../models/bear.fbx",
//             });
//             if (!ok) std::cerr << "failed to load at least one frame\n";
//             lastTime = glfwGetTime();    // gets current time for animation delta tracking
           
//         }
// private:
//     GLuint shader = 0;      // shader program ID
//     AnimatedObject anim;
//     double lastTime = 0.0;          
    
//     // custom impl of a "look-at" view matrix, equivalent to glm::lookAt()
//     // returns a matrix that transforms world-space points into camera/view space
//     Eigen::Matrix4f lookAt(
//                         const Eigen::Vector3f& eye,
//                         const Eigen::Vector3f& center,
//                         const Eigen::Vector3f& up)
//     {
//         Eigen::Vector3f f = (center - eye).normalized();
//         Eigen::Vector3f s = f.cross(up).normalized();
//         Eigen::Vector3f u = s.cross(f);

//         Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
//         m.block<1,3>(0,0) =  s.transpose();
//         m.block<1,3>(1,0) =  u.transpose();
//         m.block<1,3>(2,0) = -f.transpose();
//         m(0,3) = -s.dot(eye);
//         m(1,3) = -u.dot(eye);
//         m(2,3) =  f.dot(eye);
//         return m;
//     }

//     // key rendering loop. called automatically by nanogui.
//     virtual void drawContents() override {

//         // timing -- used for animation. updates animation state
//         double now = glfwGetTime();
//         float dt = float(now - lastTime);
//         lastTime = now;
//         anim.update(dt);



//         glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
//         glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//         glUseProgram(shader);

//         // Matrices
//         Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
//         // Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
//         Eigen::Matrix4f view =
//         lookAt(/*eye   */ { 0.0f, 10.0f, 0.0f },   //  ➜ from +Y
//             /*center*/ { 0.0f, -1.4f, 0.0f },
//             /*up    */ { 0.0f, 0.0f, 1.0f });
//         // view(2, 3) = -175.0f;
//         //view(2, 3) = -5.0f;
//         model(2, 3) = -1.4f;
//         float aspect = (float)mSize.x() / (float)mSize.y();
//         Eigen::Matrix4f proj = makePerspective(45.0f * M_PI / 180.0f, aspect, 0.1f, 100.0f);

//         // Upload uniforms
//         GLint locModel = glGetUniformLocation(shader, "uM");
//         GLint locView  = glGetUniformLocation(shader, "uV");
//         GLint locProj  = glGetUniformLocation(shader, "uP");

//         glUniformMatrix4fv(locModel, 1, GL_FALSE, model.data());
//         glUniformMatrix4fv(locView,  1, GL_FALSE, view.data());
//         glUniformMatrix4fv(locProj,  1, GL_FALSE, proj.data());
//         anim.draw();
//     }
// };


// // int main()
// // {
// //     nanogui::init();
// //     Viewer* v = new Viewer();
// //     v->setVisible(true);
// //     v->drawAll();                // first frame
// //     nanogui::mainloop();
// //     nanogui::shutdown();
// //     return 0;
// // }

// using namespace nanogui;

// int main() {
//     // Animation *animation = nullptr;
//     // GLFWwindow *window = nullptr;
//     nanogui::init();
//     Screen *screen = new Screen(Eigen::Vector2i(1024, 768), "FBX Animation Viewer", true);
//     Animation *animation = new Animation(screen);

//     screen->setVisible(true);   // show the window
//     screen->drawAll();          // render first frame
//     nanogui::mainloop();        // do the animation
//     nanogui::shutdown();
//     return 0;
// }