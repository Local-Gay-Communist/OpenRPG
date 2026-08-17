// mesh.h
#pragma once

#include <string> 

int find_closest_vertex(float x, float y, float max_dist);
void add_vertex(float x, float y);
void remove_vertex(int idx);
void add_triangle(int v0, int v1, int v2);
void compute_normals();
void update_mesh_buffers();
void duplicate_selected();

// Polygon tools
void create_square_from_selected();
void create_cube_from_selected();
void bridge_polygons();
void detriangulate_selected();

void save_project_binary(const std::string& filename);
void load_project_binary(const std::string& filename);
void export_obj(const std::string& filename);

void extrude_selected();