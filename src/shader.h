#pragma once
#include <string>
#include <glad/glad.h>

std::string loadShaderSource(const std::string& path);
GLuint compileShader(GLenum type, const std::string& src);
GLuint createShaderProgram(const std::string& vertPath, const std::string& fragPath);
