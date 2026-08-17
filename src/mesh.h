// mesh.h
#pragma once
#include <string>
#include <vector>
#include <array>

int find_closest_vertex(float x, float y, float max_dist);
void add_vertex(float x, float y);
void remove_vertex(int idx);
void add_triangle(int v0, int v1, int v2);
void compute_normals();
void update_mesh_buffers();
void duplicate_selected();

void create_square_from_selected();
void create_cube_from_selected();
void bridge_polygons();
void detriangulate_selected();

void extrude_selected();

// FIXED: signature now matches the 3-arg implementation and all call sites
void weld_vertices(float threshold = 0.001f,
                    bool weld_selected_to_center = false,
                    bool weld_selected_to_first = false);

// ----- New tools -----
void split_edge(int v0, int v1, float t);
void knife_cut(const float start[3], const float end[3]);
void unify_normals();   // fixes flipped faces

void rebuild_faces();

void save_project_binary(const std::string& filename);
void load_project_binary(const std::string& filename);
void export_obj(const std::string& filename);

void make_face_from_selected();

void split_face_by_polygon(int face_idx, const std::vector<std::array<float,3>>& points);
void get_face_boundary(int face_idx, std::vector<int>& boundary_verts);
void triangulate_polygon(const std::vector<int>& verts, std::vector<unsigned int>& out_indices);
void get_face_plane(int face_idx, float& nx, float& ny, float& nz, float& d);   // added for viewport