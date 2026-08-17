// undo.h
#pragma once
#include <vector>

struct MeshSnapshot {
    // Core mesh data
    std::vector<float> vertices;
    std::vector<float> normals;
    std::vector<unsigned int> indices;
    std::vector<int> selected;
    
    // Camera state
    float cam_theta;
    float cam_phi;
    float cam_dist;
    bool wireframe;
    
    // Lighting
    float light_azimuth;
    float light_elevation;
    float ambient_strength;

    void capture();
    void restore();
};

void push_undo();
bool can_undo();
bool can_redo();
void undo();
void redo();
void clear_undo_stack();