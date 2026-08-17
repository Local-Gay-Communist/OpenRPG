// mesh.cpp
#include "mesh.h"
#include "globals.h"
#include <GL/glew.h>
#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <cstdio>
#include <fstream>
#include <map>
#include <set>

// ---- Topology-modifying functions clear edge/face selections ----

int find_closest_vertex(float x, float y, float max_dist) {
    int best = -1;
    float best_dist = max_dist;
    for(size_t i = 0; i < g_vertices.size() / 3; i++) {
        float dx = g_vertices[i*3] - x;
        float dy = g_vertices[i*3+1] - y;
        float d = sqrtf(dx*dx + dy*dy);
        if(d < best_dist) {
            best_dist = d;
            best = (int)i;
        }
    }
    return best;
}

void add_vertex(float x, float y) {
    int snap = find_closest_vertex(x, y, SNAP_DIST);
    if(snap != -1) return;
    g_vertices.push_back(x);
    g_vertices.push_back(y);
    g_vertices.push_back(0.0f);
    g_normals.push_back(0.0f);
    g_normals.push_back(0.0f);
    g_normals.push_back(1.0f);
}

void remove_vertex(int idx) {
    // Clear edge/face selections because indices will shift
    g_selected_edges.clear();
    g_selected_faces.clear();
    
    if(idx < 0 || idx >= (int)g_vertices.size()/3) return;
    std::vector<unsigned int> new_indices;
    for(size_t i = 0; i < g_indices.size() / 3; i++) {
        unsigned int a = g_indices[i*3];
        unsigned int b = g_indices[i*3+1];
        unsigned int c = g_indices[i*3+2];
        if(a == (unsigned int)idx || b == (unsigned int)idx || c == (unsigned int)idx) continue;
        if(a > (unsigned int)idx) a--;
        if(b > (unsigned int)idx) b--;
        if(c > (unsigned int)idx) c--;
        new_indices.push_back(a);
        new_indices.push_back(b);
        new_indices.push_back(c);
    }
    g_indices = new_indices;
    g_vertices.erase(g_vertices.begin() + idx*3, g_vertices.begin() + idx*3 + 3);
    g_normals.erase(g_normals.begin() + idx*3, g_normals.begin() + idx*3 + 3);
    g_selected.erase(std::remove(g_selected.begin(), g_selected.end(), idx), g_selected.end());
    for(int& sel : g_selected) if(sel > idx) sel--;
    g_selected.erase(std::unique(g_selected.begin(), g_selected.end()), g_selected.end());
}

void add_triangle(int v0, int v1, int v2) {
    if(v0 == v1 || v0 == v2 || v1 == v2) return;
    int max_idx = (int)g_vertices.size()/3 - 1;
    if(v0 < 0 || v1 < 0 || v2 < 0 || v0 > max_idx || v1 > max_idx || v2 > max_idx) return;
    g_indices.push_back(v0);
    g_indices.push_back(v1);
    g_indices.push_back(v2);
}

void compute_normals() {
    int n = (int)g_vertices.size() / 3;
    if (n == 0) return;
    for (int i = 0; i < n; ++i) {
        g_normals[i*3] = 0.0f;
        g_normals[i*3+1] = 0.0f;
        g_normals[i*3+2] = 0.0f;
    }
    for (size_t i = 0; i < g_indices.size() / 3; ++i) {
        int a = g_indices[i*3];
        int b = g_indices[i*3+1];
        int c = g_indices[i*3+2];
        float ax = g_vertices[a*3], ay = g_vertices[a*3+1], az = g_vertices[a*3+2];
        float bx = g_vertices[b*3], by = g_vertices[b*3+1], bz = g_vertices[b*3+2];
        float cx = g_vertices[c*3], cy = g_vertices[c*3+1], cz = g_vertices[c*3+2];
        float ex1 = bx - ax, ey1 = by - ay, ez1 = bz - az;
        float ex2 = cx - ax, ey2 = cy - ay, ez2 = cz - az;
        float nx = ey1*ez2 - ez1*ey2;
        float ny = ez1*ex2 - ex1*ez2;
        float nz = ex1*ey2 - ey1*ex2;
        float len = sqrtf(nx*nx + ny*ny + nz*nz);
        if (len > 0.0001f) {
            nx /= len; ny /= len; nz /= len;
            g_normals[a*3] += nx; g_normals[a*3+1] += ny; g_normals[a*3+2] += nz;
            g_normals[b*3] += nx; g_normals[b*3+1] += ny; g_normals[b*3+2] += nz;
            g_normals[c*3] += nx; g_normals[c*3+1] += ny; g_normals[c*3+2] += nz;
        }
    }
    for (int i = 0; i < n; ++i) {
        float nx = g_normals[i*3], ny = g_normals[i*3+1], nz = g_normals[i*3+2];
        float len = sqrtf(nx*nx + ny*ny + nz*nz);
        if (len > 0.0001f) {
            g_normals[i*3] = nx / len;
            g_normals[i*3+1] = ny / len;
            g_normals[i*3+2] = nz / len;
        } else {
            g_normals[i*3] = 0.0f;
            g_normals[i*3+1] = 0.0f;
            g_normals[i*3+2] = 1.0f;
        }
    }
}

void update_mesh_buffers() {
    compute_normals();
    g_vert_count = (int)g_vertices.size() / 3;
    g_idx_count = (int)g_indices.size();

    std::vector<float> colors;
    colors.reserve(g_vertices.size());
    for(size_t i = 0; i < g_vertices.size() / 3; i++) {
        bool sel = std::find(g_selected.begin(), g_selected.end(), (int)i) != g_selected.end();
        if(sel) {
            colors.push_back(1.0f); colors.push_back(1.0f); colors.push_back(0.0f);
        } else {
            colors.push_back(0.7f); colors.push_back(0.7f); colors.push_back(0.8f);
        }
    }

    std::vector<float> interleaved;
    interleaved.reserve(g_vertices.size() * 3 + g_normals.size() + colors.size());
    for(size_t i = 0; i < g_vertices.size() / 3; i++) {
        interleaved.push_back(g_vertices[i*3]);
        interleaved.push_back(g_vertices[i*3+1]);
        interleaved.push_back(g_vertices[i*3+2]);
        interleaved.push_back(g_normals[i*3]);
        interleaved.push_back(g_normals[i*3+1]);
        interleaved.push_back(g_normals[i*3+2]);
        interleaved.push_back(colors[i*3]);
        interleaved.push_back(colors[i*3+1]);
        interleaved.push_back(colors[i*3+2]);
    }

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, interleaved.size() * sizeof(float), interleaved.data(), GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, g_indices.size() * sizeof(unsigned int), g_indices.data(), GL_DYNAMIC_DRAW);
}

// ---- Polygon Tools ----
void create_square_from_selected() {
    g_selected_edges.clear();
    g_selected_faces.clear();
    if (g_selected.size() < 2) return;
    int idx1 = g_selected[0], idx2 = g_selected[1];
    float x1 = g_vertices[idx1*3], y1 = g_vertices[idx1*3+1];
    float x2 = g_vertices[idx2*3], y2 = g_vertices[idx2*3+1];
    float x3 = x2, y3 = y1;
    float x4 = x1, y4 = y2;
    int base = g_vertices.size()/3;
    add_vertex(x1, y1);
    add_vertex(x2, y1);
    add_vertex(x2, y2);
    add_vertex(x1, y2);
    add_triangle(base, base+1, base+2);
    add_triangle(base, base+2, base+3);
    g_selected.clear();
    g_selected.push_back(base);
    g_selected.push_back(base+1);
    g_selected.push_back(base+2);
    g_selected.push_back(base+3);
    update_mesh_buffers();
}

void create_cube_from_selected() {
    g_selected_edges.clear();
    g_selected_faces.clear();
    if (g_selected.empty()) return;
    int idx = g_selected[0];
    float x = g_vertices[idx*3], y = g_vertices[idx*3+1], z = g_vertices[idx*3+2];
    float s = 0.5f;
    float verts[8][3] = {
        {x, y, z},
        {x+s, y, z},
        {x+s, y+s, z},
        {x, y+s, z},
        {x, y, z+s},
        {x+s, y, z+s},
        {x+s, y+s, z+s},
        {x, y+s, z+s}
    };
    int base = g_vertices.size()/3;
    for (int i=0; i<8; i++) {
        g_vertices.push_back(verts[i][0]);
        g_vertices.push_back(verts[i][1]);
        g_vertices.push_back(verts[i][2]);
        g_normals.push_back(0.0f);
        g_normals.push_back(0.0f);
        g_normals.push_back(0.0f);
    }
    int faces[6][4] = {
        {0,1,2,3}, // front
        {5,4,7,6}, // back
        {0,4,5,1}, // bottom
        {3,2,6,7}, // top
        {0,3,7,4}, // left
        {1,5,6,2}  // right
    };
    for (int f=0; f<6; f++) {
        int a = base + faces[f][0];
        int b = base + faces[f][1];
        int c = base + faces[f][2];
        int d = base + faces[f][3];
        add_triangle(a, b, c);
        add_triangle(a, c, d);
    }
    g_selected.clear();
    for (int i=0; i<8; i++) g_selected.push_back(base+i);
    update_mesh_buffers();
}

void bridge_polygons() {
    g_selected_edges.clear();
    g_selected_faces.clear();
    if (g_selected.size() < 6) return;
    size_t half = g_selected.size() / 2;
    std::vector<int> polyA, polyB;
    for (size_t i=0; i<half; i++) polyA.push_back(g_selected[i]);
    for (size_t i=half; i<g_selected.size(); i++) polyB.push_back(g_selected[i]);
    if (polyA.size() < 3 || polyB.size() < 3) return;
    size_t nA = polyA.size(), nB = polyB.size();
    for (size_t i=0; i<nA; i++) {
        int a0 = polyA[i], a1 = polyA[(i+1)%nA];
        int b0 = polyB[i % nB], b1 = polyB[(i+1)%nB];
        add_triangle(a0, a1, b0);
        add_triangle(a1, b1, b0);
    }
    g_selected.clear();
    update_mesh_buffers();
}

void detriangulate_selected() {
    g_selected_edges.clear();
    g_selected_faces.clear();
    if (g_selected.empty()) return;
    std::vector<int> tris_to_remove;
    std::vector<int> unique_verts;
    for (size_t i = 0; i < g_indices.size() / 3; i++) {
        int a = g_indices[i*3], b = g_indices[i*3+1], c = g_indices[i*3+2];
        bool all_selected = true;
        for (int v : {a, b, c}) {
            if (std::find(g_selected.begin(), g_selected.end(), v) == g_selected.end()) {
                all_selected = false;
                break;
            }
        }
        if (all_selected) {
            tris_to_remove.push_back(i);
            for (int v : {a, b, c}) {
                if (std::find(unique_verts.begin(), unique_verts.end(), v) == unique_verts.end())
                    unique_verts.push_back(v);
            }
        }
    }
    if (tris_to_remove.empty() || unique_verts.size() < 3) return;
    std::sort(tris_to_remove.begin(), tris_to_remove.end(), std::greater<int>());
    for (int idx : tris_to_remove) {
        g_indices.erase(g_indices.begin() + idx*3, g_indices.begin() + idx*3 + 3);
    }
    float cx = 0, cy = 0, cz = 0;
    for (int v : unique_verts) {
        cx += g_vertices[v*3]; cy += g_vertices[v*3+1]; cz += g_vertices[v*3+2];
    }
    cx /= unique_verts.size(); cy /= unique_verts.size(); cz /= unique_verts.size();
    std::sort(unique_verts.begin(), unique_verts.end(),
        [&](int a, int b) {
            float ax = g_vertices[a*3] - cx, ay = g_vertices[a*3+1] - cy;
            float bx = g_vertices[b*3] - cx, by = g_vertices[b*3+1] - cy;
            return atan2(ay, ax) < atan2(by, bx);
        });
    int first = unique_verts[0];
    for (size_t i = 1; i + 1 < unique_verts.size(); i++) {
        add_triangle(first, unique_verts[i], unique_verts[i+1]);
    }
    g_selected.clear();
    update_mesh_buffers();
}

void duplicate_selected() {
    g_selected_edges.clear();
    g_selected_faces.clear();
    if (g_selected.empty()) return;
    std::map<int, int> remap;
    float offset_x = 0.2f, offset_y = 0.2f;
    for (int old_idx : g_selected) {
        float x = g_vertices[old_idx*3] + offset_x;
        float y = g_vertices[old_idx*3+1] + offset_y;
        float z = g_vertices[old_idx*3+2];
        g_vertices.push_back(x);
        g_vertices.push_back(y);
        g_vertices.push_back(z);
        g_normals.push_back(0.0f);
        g_normals.push_back(0.0f);
        g_normals.push_back(1.0f);
        int new_idx = (int)g_vertices.size()/3 - 1;
        remap[old_idx] = new_idx;
    }
    for (size_t i = 0; i < g_indices.size() / 3; i++) {
        unsigned int a = g_indices[i*3];
        unsigned int b = g_indices[i*3+1];
        unsigned int c = g_indices[i*3+2];
        if (remap.find((int)a) != remap.end() &&
            remap.find((int)b) != remap.end() &&
            remap.find((int)c) != remap.end()) {
            int na = remap[(int)a];
            int nb = remap[(int)b];
            int nc = remap[(int)c];
            g_indices.push_back(na);
            g_indices.push_back(nb);
            g_indices.push_back(nc);
        }
    }
    g_selected.clear();
    for (auto& pair : remap) g_selected.push_back(pair.second);
    update_mesh_buffers();
}

// ---- Save/Load/Export ----
void save_project_binary(const std::string& filename) {
    FILE* f = fopen(filename.c_str(), "wb");
    if (!f) { printf("Failed to open %s for writing\n", filename.c_str()); return; }
    uint32_t magic = 0x475052, version = 1;
    uint32_t vc = (uint32_t)(g_vertices.size()/3);
    uint32_t nc = (uint32_t)(g_normals.size()/3);
    uint32_t ic = (uint32_t)g_indices.size();
    uint32_t sc = (uint32_t)g_selected.size();
    fwrite(&magic, 4, 1, f);
    fwrite(&version, 4, 1, f);
    fwrite(&vc, 4, 1, f);
    fwrite(&nc, 4, 1, f);
    fwrite(&ic, 4, 1, f);
    fwrite(&sc, 4, 1, f);
    fwrite(&cam_theta, sizeof(float), 1, f);
    fwrite(&cam_phi, sizeof(float), 1, f);
    fwrite(&cam_dist, sizeof(float), 1, f);
    fwrite(&wireframe, sizeof(bool), 1, f);
    fwrite(g_vertices.data(), sizeof(float), g_vertices.size(), f);
    fwrite(g_normals.data(), sizeof(float), g_normals.size(), f);
    fwrite(g_indices.data(), sizeof(uint32_t), g_indices.size(), f);
    fwrite(g_selected.data(), sizeof(uint32_t), g_selected.size(), f);
    fclose(f);
    printf("Project saved to %s (%u vertices, %u triangles)\n", filename.c_str(), vc, ic/3);
}

void load_project_binary(const std::string& filename) {
    FILE* f = fopen(filename.c_str(), "rb");
    if (!f) { printf("Failed to open %s for reading\n", filename.c_str()); return; }
    uint32_t magic, version, vc, nc, ic, sc;
    fread(&magic, 4, 1, f);
    if (magic != 0x475052) { printf("Invalid file format\n"); fclose(f); return; }
    fread(&version, 4, 1, f);
    if (version != 1) { printf("Unsupported version\n"); fclose(f); return; }
    fread(&vc, 4, 1, f);
    fread(&nc, 4, 1, f);
    fread(&ic, 4, 1, f);
    fread(&sc, 4, 1, f);
    fread(&cam_theta, sizeof(float), 1, f);
    fread(&cam_phi, sizeof(float), 1, f);
    fread(&cam_dist, sizeof(float), 1, f);
    fread(&wireframe, sizeof(bool), 1, f);
    g_vertices.clear(); g_normals.clear(); g_indices.clear(); g_selected.clear();
    g_selected_edges.clear();
    g_selected_faces.clear();
    g_vertices.resize(vc*3);
    g_normals.resize(nc*3);
    g_indices.resize(ic);
    g_selected.resize(sc);
    fread(g_vertices.data(), sizeof(float), g_vertices.size(), f);
    fread(g_normals.data(), sizeof(float), g_normals.size(), f);
    fread(g_indices.data(), sizeof(uint32_t), g_indices.size(), f);
    fread(g_selected.data(), sizeof(uint32_t), g_selected.size(), f);
    fclose(f);
    update_mesh_buffers();
    printf("Project loaded from %s (%u vertices, %u triangles)\n", filename.c_str(), vc, ic/3);
}

void export_obj(const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) { printf("Failed to open %s for writing\n", filename.c_str()); return; }
    file << "# Exported from OpenRPG Workshop\n";
    file << "o mesh\n";
    for (size_t i = 0; i < g_vertices.size()/3; i++)
        file << "v " << g_vertices[i*3] << " " << g_vertices[i*3+1] << " " << g_vertices[i*3+2] << "\n";
    for (size_t i = 0; i < g_normals.size()/3; i++)
        file << "vn " << g_normals[i*3] << " " << g_normals[i*3+1] << " " << g_normals[i*3+2] << "\n";
    for (size_t i = 0; i < g_indices.size()/3; i++) {
        int a = g_indices[i*3]+1, b = g_indices[i*3+1]+1, c = g_indices[i*3+2]+1;
        file << "f " << a << "//" << a << " " << b << "//" << b << " " << c << "//" << c << "\n";
    }
    file.close();
    printf("Exported OBJ to %s\n", filename.c_str());
}

void extrude_selected() {
    if (g_selected_faces.empty()) return;
    
    // ---- 1. Collect unique vertices from selected faces ----
    std::set<int> vert_set;
    for (int tri : g_selected_faces) {
        vert_set.insert(g_indices[tri*3]);
        vert_set.insert(g_indices[tri*3+1]);
        vert_set.insert(g_indices[tri*3+2]);
    }
    if (vert_set.empty()) return;
    
    // ---- 2. Compute average normal of selected faces ----
    float avg_nx = 0, avg_ny = 0, avg_nz = 0;
    for (int tri : g_selected_faces) {
        int a = g_indices[tri*3], b = g_indices[tri*3+1], c = g_indices[tri*3+2];
        float ax = g_vertices[a*3], ay = g_vertices[a*3+1], az = g_vertices[a*3+2];
        float bx = g_vertices[b*3], by = g_vertices[b*3+1], bz = g_vertices[b*3+2];
        float cx = g_vertices[c*3], cy = g_vertices[c*3+1], cz = g_vertices[c*3+2];
        float ex1 = bx-ax, ey1 = by-ay, ez1 = bz-az;
        float ex2 = cx-ax, ey2 = cy-ay, ez2 = cz-az;
        float nx = ey1*ez2 - ez1*ey2;
        float ny = ez1*ex2 - ex1*ez2;
        float nz = ex1*ey2 - ey1*ex2;
        float len = sqrtf(nx*nx + ny*ny + nz*nz);
        if (len > 0.0001f) {
            avg_nx += nx / len;
            avg_ny += ny / len;
            avg_nz += nz / len;
        }
    }
    float alen = sqrtf(avg_nx*avg_nx + avg_ny*avg_ny + avg_nz*avg_nz);
    if (alen > 0.0001f) {
        avg_nx /= alen; avg_ny /= alen; avg_nz /= alen;
    } else {
        avg_nz = 1.0f; // fallback
    }
    
    float offset = 0.5f; // extrusion distance (could be made configurable later)
    
    // ---- 3. Duplicate vertices and map old->new ----
    std::map<int, int> remap;
    for (int old_idx : vert_set) {
        float new_x = g_vertices[old_idx*3] + avg_nx * offset;
        float new_y = g_vertices[old_idx*3+1] + avg_ny * offset;
        float new_z = g_vertices[old_idx*3+2] + avg_nz * offset;
        int new_idx = (int)g_vertices.size() / 3;
        g_vertices.push_back(new_x);
        g_vertices.push_back(new_y);
        g_vertices.push_back(new_z);
        g_normals.push_back(0.0f);
        g_normals.push_back(0.0f);
        g_normals.push_back(1.0f); // placeholder, will be recomputed
        remap[old_idx] = new_idx;
    }
    
    // ---- 4. Build edge count map (to detect boundaries) ----
    std::map<std::pair<int,int>, int> edge_count;
    for (int tri : g_selected_faces) {
        int v[3] = {g_indices[tri*3], g_indices[tri*3+1], g_indices[tri*3+2]};
        for (int i = 0; i < 3; i++) {
            int a = v[i], b = v[(i+1)%3];
            if (a > b) std::swap(a, b);
            edge_count[{a, b}]++;
        }
    }
    
    // ---- 5. For each selected face, create extruded face and side quads ----
    for (int tri : g_selected_faces) {
        int v[3] = {g_indices[tri*3], g_indices[tri*3+1], g_indices[tri*3+2]};
        int new_v[3] = {remap[v[0]], remap[v[1]], remap[v[2]]};
        
        // Extruded face (new triangle)
        g_indices.push_back(new_v[0]);
        g_indices.push_back(new_v[1]);
        g_indices.push_back(new_v[2]);
        
        // Side quads for boundary edges
        for (int i = 0; i < 3; i++) {
            int a = v[i], b = v[(i+1)%3];
            int a_orig = a, b_orig = b;
            if (a_orig > b_orig) std::swap(a_orig, b_orig);
            if (edge_count[{a_orig, b_orig}] == 1) {
                // Boundary edge: create quad (a, b, nb, na)
                int na = remap[a];
                int nb = remap[b];
                g_indices.push_back(a);
                g_indices.push_back(b);
                g_indices.push_back(nb);
                g_indices.push_back(a);
                g_indices.push_back(nb);
                g_indices.push_back(na);
            }
        }
    }
    
    // ---- 6. Clean up selection and refresh ----
    g_selected_faces.clear();
    g_selected_edges.clear(); // just in case
    g_selected.clear();
    update_mesh_buffers();
    update_gizmo_selection();
}