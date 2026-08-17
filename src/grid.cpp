// grid.cpp – updated to include normals
#include "grid.h"
#include "globals.h"
#include <GL/glew.h>
#include <vector>
#include <cmath>

static unsigned int grid_VAO, grid_VBO;
static int grid_vertex_count = 0;

void init_grid() {
    const int half = 10;
    const float step = 1.0f;
    const float major_step = 5.0f;
    std::vector<float> vertices; // x,y,z, nx,ny,nz, r,g,b

    auto add_line = [&](float x1, float y1, float x2, float y2, float r, float g, float b) {
        // Each vertex: pos, normal(0,0,1), color
        vertices.push_back(x1); vertices.push_back(y1); vertices.push_back(0.0f);
        vertices.push_back(0.0f); vertices.push_back(0.0f); vertices.push_back(1.0f);
        vertices.push_back(r); vertices.push_back(g); vertices.push_back(b);
        vertices.push_back(x2); vertices.push_back(y2); vertices.push_back(0.0f);
        vertices.push_back(0.0f); vertices.push_back(0.0f); vertices.push_back(1.0f);
        vertices.push_back(r); vertices.push_back(g); vertices.push_back(b);
    };

    for (float y = -half; y <= half; y += step) {
        bool major = (fmod(fabs(y), major_step) < 0.001f);
        float r = major ? 0.35f : 0.15f;
        float g = major ? 0.35f : 0.15f;
        float b = major ? 0.35f : 0.15f;
        add_line(-half, y, half, y, r, g, b);
    }
    for (float x = -half; x <= half; x += step) {
        bool major = (fmod(fabs(x), major_step) < 0.001f);
        float r = major ? 0.35f : 0.15f;
        float g = major ? 0.35f : 0.15f;
        float b = major ? 0.35f : 0.15f;
        add_line(x, -half, x, half, r, g, b);
    }

    grid_vertex_count = (int)vertices.size() / 9;

    glGenVertexArrays(1, &grid_VAO);
    glGenBuffers(1, &grid_VBO);
    glBindVertexArray(grid_VAO);
    glBindBuffer(GL_ARRAY_BUFFER, grid_VBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);

    // Position (location 0)
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    // Normal (location 1)
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    // Color (location 2)
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(float), (void*)(6 * sizeof(float)));
    glEnableVertexAttribArray(2);

    glBindVertexArray(0);
}

void render_grid(const Mat4& mvp) {
    if (grid_vertex_count == 0) return;
    glUseProgram(shaderProgram);
    int mvpLoc = glGetUniformLocation(shaderProgram, "mvp");
    if (mvpLoc != -1) glUniformMatrix4fv(mvpLoc, 1, GL_FALSE, mvp.m);
    // For grid, we want flat shading; model matrix is identity, but we need to set it.
    // We'll set model to identity and lightDir to some default.
    int modelLoc = glGetUniformLocation(shaderProgram, "model");
    if (modelLoc != -1) {
        Mat4 ident = identity();
        glUniformMatrix4fv(modelLoc, 1, GL_FALSE, ident.m);
    }
    int lightDirLoc = glGetUniformLocation(shaderProgram, "lightDir");
    if (lightDirLoc != -1) {
        glUniform3f(lightDirLoc, 0.5f, 0.8f, 0.3f);
    }
    glBindVertexArray(grid_VAO);
    glDrawArrays(GL_LINES, 0, grid_vertex_count);
    glBindVertexArray(0);
}