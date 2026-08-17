// shader.cpp
#include "shader.h"
#include <GL/glew.h>
#include <cstdio>

const char* vertexShaderSource = R"(
#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;
layout (location = 2) in vec3 aColor;

uniform mat4 mvp;
uniform mat4 model;

out vec3 Color;
out vec3 Normal;
out vec3 FragPos;

void main() {
    FragPos = vec3(model * vec4(aPos, 1.0));
    Normal = normalize(mat3(transpose(inverse(model))) * aNormal);
    Color = aColor;
    gl_Position = mvp * vec4(aPos, 1.0);
}
)";

const char* fragmentShaderSource = R"(
#version 330 core
in vec3 Color;
in vec3 Normal;
in vec3 FragPos;

out vec4 FragColor;

uniform vec3 lightDir;
uniform float ambientStrength;

void main() {
    vec3 ambient = ambientStrength * Color;
    vec3 norm = normalize(Normal);
    float diff = max(dot(norm, lightDir), 0.0);
    vec3 diffuse = diff * Color;
    FragColor = vec4(ambient + diffuse, 1.0);
}
)";

unsigned int compile_shader(const char* source, unsigned int type) {
    unsigned int shader = glCreateShader(type);
    glShaderSource(shader, 1, &source, NULL);
    glCompileShader(shader);
    int success;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if(!success) {
        char log[512];
        glGetShaderInfoLog(shader, 512, NULL, log);
        printf("Shader error: %s\n", log);
    }
    return shader;
}

unsigned int create_program() {
    unsigned int vs = compile_shader(vertexShaderSource, GL_VERTEX_SHADER);
    unsigned int fs = compile_shader(fragmentShaderSource, GL_FRAGMENT_SHADER);
    unsigned int prog = glCreateProgram();
    glAttachShader(prog, vs);
    glAttachShader(prog, fs);
    glLinkProgram(prog);
    int success;
    glGetProgramiv(prog, GL_LINK_STATUS, &success);
    if(!success) {
        char log[512];
        glGetProgramInfoLog(prog, 512, NULL, log);
        printf("Program link error: %s\n", log);
    }
    glDeleteShader(vs);
    glDeleteShader(fs);
    return prog;
}