#include "shader.h"
#include <fstream>
#include <sstream>
#include <iostream>

std::string loadShaderSource(const std::string& path) {
    std::ifstream file(path);
    std::stringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
}

GLuint compileShader(GLenum type, const std::string& src) {
    GLuint shader = glCreateShader(type);
    const char* cstr = src.c_str();
    glShaderSource(shader, 1, &cstr, nullptr);
    glCompileShader(shader);

    GLint success;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        char info[512];
        glGetShaderInfoLog(shader, 512, nullptr, info);
        std::cerr << "Shader compile error:\n" << info << std::endl;
    }

    return shader;
}

GLuint createShaderProgram(const std::string& vertPath, const std::string& fragPath) {
    std::string vertCode = loadShaderSource(vertPath);
    std::string fragCode = loadShaderSource(fragPath);

    GLuint vertShader = compileShader(GL_VERTEX_SHADER, vertCode);
    GLuint fragShader = compileShader(GL_FRAGMENT_SHADER, fragCode);

    GLuint program = glCreateProgram();
    glAttachShader(program, vertShader);
    glAttachShader(program, fragShader);
    glLinkProgram(program);

    GLint success;
    glGetProgramiv(program, GL_LINK_STATUS, &success);
    if (!success) {
        char info[512];
        glGetProgramInfoLog(program, 512, nullptr, info);
        std::cerr << "Shader link error:\n" << info << std::endl;
    }

    glDeleteShader(vertShader);
    glDeleteShader(fragShader);

    return program;
}
