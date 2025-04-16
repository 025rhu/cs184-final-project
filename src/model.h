#pragma once
#include <string>
#include <glad/glad.h>

extern GLuint VAO, VBO, NBO, CBO, EBO;
extern int indexCount;

void loadModel(const std::string& path);
