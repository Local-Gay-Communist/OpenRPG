// globals.h
#pragma once
#include <vector>
#include <utility>
#include <array>    // for std::array

// Mesh data
extern std::vector<float> g_vertices;
extern std::vector<float> g_normals;
extern std::vector<unsigned int> g_indices;
extern std::vector<int> g_selected;
extern int g_vert_count;
extern int g_idx_count;
extern float SNAP_DIST;
extern float g_last_ground_x;
extern float g_last_ground_y;

// OpenGL buffers
extern unsigned int VAO, VBO, EBO;

// FBO
extern unsigned int fbo_texture, fbo_renderbuffer, fbo;
extern int viewport_width, viewport_height;

// Shader program
extern unsigned int shaderProgram;

// Camera orbit
extern float cam_theta, cam_phi, cam_dist;

// Flags
extern bool wireframe;

// Lighting
extern float light_azimuth;
extern float light_elevation;
extern float ambient_strength;

// Selection Mode
enum SelectionMode {
    SELECT_VERTEX = 1,
    SELECT_EDGE   = 2,
    SELECT_FACE   = 3
};
extern SelectionMode g_sel_mode;

// Edge and Face selection storage
extern std::vector<std::pair<int, int>> g_selected_edges;
extern std::vector<int> g_selected_faces;

// Gizmo state
extern bool g_gizmo_enabled;
extern int g_gizmo_axis;          // -1 = none, 0 = X, 1 = Y, 2 = Z
extern float g_gizmo_pos[3];
extern std::vector<int> g_gizmo_verts;
extern float g_gizmo_size;

struct MeshFace {
    std::vector<int> tri_indices; // triangle indices in g_indices
};
extern std::vector<MeshFace> g_face_list;

extern bool g_knife_active;
extern int g_knife_stage;      // 0=idle, 1=first point set, 2=second point set
extern float g_knife_start[3];
extern float g_knife_end[3];

// ---- Draw‑Face state ----
extern bool g_draw_face_active;
extern int g_draw_face_face_idx;        // face being cut
extern std::vector<std::array<float,3>> g_draw_face_points;   // clicked points (3D)
