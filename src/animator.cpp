#include "animator.h"
#include "Eigen/src/Geometry/Quaternion.h"
#include "nanogui/common.h"
#include <iostream>
#include <vector>
// #include <glad/glad.h>
// #include "CGL/CGL.h"
// #include <CGL/vector3D.h>
// #include <iostream>
// #include <set>
// #include <string>
// #include "nanogui/glutil.h"
// #include "utils.h"


using namespace std;

void Bone::interpolateAt(double time, Matrix4f parentTransform) {
    Matrix4f localTransform = buildLocalTransform(time);
    Matrix4f globalTransform = localTransform * parentTransform;    // try parent * local if this looks wonky
    this->globalTransformation = globalTransform;
    
    for (auto child : *children) {
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
    if (time >= positionTimes[-1]){
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
    if (time >= rotationTimes[-1]){
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
    if (time >= scalingTimes[-1]){
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
// KeyFrameAnimator::KeyFrameAnimator(std::string project_root, Screen *screen)
// : m_project_root(project_root) {
//   this->screen = screen;
  
//   this->loadShaders();
// //   this->load_textures();

// //   glEnable(GL_PROGRAM_POINT_SIZE);
//   // enables depth buffering so closer objects occlude further ones correctly
//   glEnable(GL_DEPTH_TEST);
// }


// KeyFrameAnimator::~KeyFrameAnimator() {
//     // for (auto shader : shaders) {
//     shader->free();
//     // }
//     // glDeleteTextures(1, &m_gl_texture_1);
//     // glDeleteTextures(1, &m_gl_texture_2);
//     // glDeleteTextures(1, &m_gl_texture_3);
//     // glDeleteTextures(1, &m_gl_texture_4);
//     // glDeleteTextures(1, &m_gl_texture_5);
  
//     // glDeleteTextures(1, &m_gl_cubemap_tex);
  
//     // if (cloth) delete cloth;
//     // if (cp) delete cp;
//     // if (collision_objects) delete collision_objects;
//     // TODO: add code to free the object
//   }


// /**
//  * Initializes the cloth simulation and spawns a new thread to separate
//  * rendering from simulation.
//  */
//  void KeyFrameAnimator::init() {

//     // Initialize GUI
//     screen->setSize(default_window_size);
//     initGUI(screen);
  
//     // // Initialize camera
  
//     // CGL::Collada::CameraInfo camera_info;
//     // camera_info.hFov = 50;
//     // camera_info.vFov = 35;
//     // camera_info.nClip = 0.01;
//     // camera_info.fClip = 10000;
  
//     // // Try to intelligently figure out the camera target
  
//     // Vector3D avg_pm_position(0, 0, 0);
  
//     // for (auto &pm : cloth->point_masses) {
//     //   avg_pm_position += pm.position / cloth->point_masses.size();
//     // }
  
//     // CGL::Vector3D target(avg_pm_position.x, avg_pm_position.y / 2,
//     //                      avg_pm_position.z);
//     // CGL::Vector3D c_dir(0., 0., 0.);
//     // canonical_view_distance = max(cloth->width, cloth->height) * 0.9;
//     // scroll_rate = canonical_view_distance / 10;
  
//     // view_distance = canonical_view_distance * 2;
//     // min_view_distance = canonical_view_distance / 10.0;
//     // max_view_distance = canonical_view_distance * 20.0;
  
//     // // canonicalCamera is a copy used for view resets
  
//     // camera.place(target, acos(c_dir.y), atan2(c_dir.x, c_dir.z), view_distance,
//     //              min_view_distance, max_view_distance);
//     // canonicalCamera.place(target, acos(c_dir.y), atan2(c_dir.x, c_dir.z),
//     //                       view_distance, min_view_distance, max_view_distance);
  
//     // screen_w = default_window_size(0);
//     // screen_h = default_window_size(1);
  
//     // camera.configure(camera_info, screen_w, screen_h);
//     // canonicalCamera.configure(camera_info, screen_w, screen_h);
//   }
  

// //   void KeyFrameAnimator::loadShadersComplex() {
// //     // read all filenames from /shaders directory
// //     std::set<std::string> shader_filenames;
// //     bool success = Utils::list_files_in_directory(m_project_root + "/shaders", shader_filenames);
// //     if (!success) {
// //       std::cout << "Error: Could not find the shaders folder!" << std::endl;
// //     }
    
// //     std::string std_vert_shader = m_project_root + "/shaders/Default.vert";
    
// //     // iterate through all filenames
// //     for (const std::string& shader_fname : shader_filenames) {
// //         // validate the shader
// //         std::string file_extension;
// //         std::string shader_name;
        
// //         Utils::split_filename(shader_fname, shader_name, file_extension);
        
// //         if (file_extension != "frag") {
// //             std::cout << "Skipping non-shader file: " << shader_fname << std::endl;
// //             continue;
// //         }
        
// //         std::cout << "Found shader file: " << shader_fname << std::endl;
        
// //         // Check if there is a proper .vert shader or not for it. If not, use Default.vert
// //         std::string vert_shader = std_vert_shader;
// //         std::string associated_vert_shader_path = m_project_root + "/shaders/" + shader_name + ".vert";
// //         if (Utils::file_exists(associated_vert_shader_path)) {
// //             vert_shader = associated_vert_shader_path;
// //         }
      
// //       std::shared_ptr<GLShader> nanogui_shader = make_shared<GLShader>();
// //       nanogui_shader->initFromFiles(shader_name, vert_shader,
// //                                     m_project_root + "/shaders/" + shader_fname);
      
// //       // Special filenames are treated a bit differently
// //     //   ShaderTypeHint hint;
// //     //   if (shader_name == "Wireframe") {
// //     //     hint = ShaderTypeHint::WIREFRAME;
// //     //     std::cout << "Type: Wireframe" << std::endl;
// //     //   } else if (shader_name == "Normal") {
// //     //     hint = ShaderTypeHint::NORMALS;
// //     //     std::cout << "Type: Normal" << std::endl;
// //     //   } else {
// //     //     hint = ShaderTypeHint::PHONG;
// //     //     std::cout << "Type: Custom" << std::endl;
// //     //   }
      
// //     //   UserShader user_shader(shader_name, nanogui_shader, hint);
      
// //       shaders.push_back(nanogui_shader);
// //       shaders_combobox_names.push_back(shader_name);
// //     }
    
// //     // Assuming that it's there, use "Wireframe" by default
// //     for (size_t i = 0; i < shaders_combobox_names.size(); ++i) {
// //       if (shaders_combobox_names[i] == "Wireframe") {
// //         active_shader_idx = i;
// //         break;
// //       }
// //     }
// //   }


// void KeyFrameAnimator::loadShaders() {
//     // TODO: update this. unnecessary reading all filenames...read all filenames from /shaders directory
//     std::set<std::string> shader_filenames;
//     bool success = Utils::list_files_in_directory(m_project_root + "/shaders", shader_filenames);
//     if (!success) {
//       std::cout << "Error: Could not find the shaders folder!" << std::endl;
//     }
    
//     std::string shader_name = "Default";
//     std::string vert_shader_fname = m_project_root + "/shaders/Default.vert";
//     std::string frag_shader_fname = m_project_root + "/shaders/Default.frag";
//     shader->initFromFiles(shader_name, vert_shader_fname, frag_shader_fname);
//   }
    