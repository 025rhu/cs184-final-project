// #include "animator.h"
// #include <glad/glad.h>
// #include "CGL/CGL.h"
// #include <CGL/vector3D.h>
// #include <iostream>
// #include <set>
// #include <string>
// #include "nanogui/glutil.h"
// #include "utils.h"


// using namespace CGL;
// using namespace std;

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
    