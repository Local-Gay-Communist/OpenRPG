// undo.cpp
#include "undo.h"
#include "globals.h"
#include "mesh.h"
#include <cstdio>
#include <algorithm>

static std::vector<MeshSnapshot> undo_stack;
static std::vector<MeshSnapshot> redo_stack;
static const int MAX_UNDO = 20;

void MeshSnapshot::capture() {
    vertices = g_vertices;
    normals = g_normals;
    indices = g_indices;
    selected = g_selected;
    
    cam_theta = ::cam_theta;
    cam_phi = ::cam_phi;
    cam_dist = ::cam_dist;
    wireframe = ::wireframe;
    
    light_azimuth = ::light_azimuth;
    light_elevation = ::light_elevation;
    ambient_strength = ::ambient_strength;
}

void MeshSnapshot::restore() {
    g_vertices = vertices;
    g_normals = normals;
    g_indices = indices;
    g_selected = selected;
    
    ::cam_theta = cam_theta;
    ::cam_phi = cam_phi;
    ::cam_dist = cam_dist;
    ::wireframe = wireframe;
    
    ::light_azimuth = light_azimuth;
    ::light_elevation = light_elevation;
    ::ambient_strength = ambient_strength;
    
    // Rebuild OpenGL buffers
    update_mesh_buffers();
}

void push_undo() {
    // Pushing a new state invalidates the redo stack
    if (!redo_stack.empty()) {
        redo_stack.clear();
    }
    
    // If stack is full, remove the oldest (front)
    if (undo_stack.size() >= MAX_UNDO) {
        undo_stack.erase(undo_stack.begin());
    }
    
    MeshSnapshot snapshot;
    snapshot.capture();
    undo_stack.push_back(snapshot);
}

bool can_undo() {
    return !undo_stack.empty();
}

bool can_redo() {
    return !redo_stack.empty();
}

void undo() {
    if (!can_undo()) return;
    
    // Save current state to redo stack
    MeshSnapshot current;
    current.capture();
    redo_stack.push_back(current);
    
    // Restore from undo stack
    MeshSnapshot snapshot = undo_stack.back();
    undo_stack.pop_back();
    snapshot.restore();
    
    printf("Undo: %zu steps left\n", undo_stack.size());
}

void redo() {
    if (!can_redo()) return;
    
    // Save current state to undo stack
    MeshSnapshot current;
    current.capture();
    undo_stack.push_back(current);
    
    // Restore from redo stack
    MeshSnapshot snapshot = redo_stack.back();
    redo_stack.pop_back();
    snapshot.restore();
    
    printf("Redo: %zu steps left\n", redo_stack.size());
}

void clear_undo_stack() {
    undo_stack.clear();
    redo_stack.clear();
    printf("Undo/Redo stacks cleared\n");
}