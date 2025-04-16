#ifndef KEYFRAME_ANIMATOR_H
#define KEYFRAME_ANIMATOR_H

#include "nanogui/glutil.h"
#include "string"
#include <nanogui/screen.h>

using namespace nanogui;
using namespace std;

class KeyFrameAnimator {
public:
    KeyFrameAnimator(std::string project_root, Screen *screen);
    ~KeyFrameAnimator();

    void init();

    void loadModel();
    virtual void drawContents();

private:
    virtual void initGUI(Screen *screen);
    void loadShaders();

    // Camera methods

    virtual void resetCamera();
    virtual Matrix4f getProjectionMatrix();
    virtual Matrix4f getViewMatrix();

    // File management

    std::string m_project_root;

    // Screen methods
    Screen *screen;

    // OpenGL attributes

    int active_shader_idx = 0;

    GLShader* shader;
    // vector<GLShader*> shaders;
    // GLShader* vert_shader;
    // GLShader* frag_shader;

    // vector<std::string> shaders_combobox_names;

    // Screen attributes

    Vector2i default_window_size = Vector2i(1024, 800);


};

#endif // KEYFRAME_ANIMATOR_H