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
#include "viewport.h"
#include <array>   
#include <queue>

// ---- Topology-modifying functions clear edge/face selections ----

void get_face_boundary(int face_idx, std::vector<int>& boundary_verts) {
    boundary_verts.clear();
    if (face_idx < 0 || face_idx >= (int)g_face_list.size()) return;

    MeshFace& face = g_face_list[face_idx];
    std::map<std::pair<int,int>, int> edge_count;
    for (int tri : face.tri_indices) {
        int a = g_indices[tri*3], b = g_indices[tri*3+1], c = g_indices[tri*3+2];
        int edges[3][2] = {{a,b},{b,c},{c,a}};
        for (int j = 0; j < 3; ++j) {
            int u = std::min(edges[j][0], edges[j][1]);
            int v = std::max(edges[j][0], edges[j][1]);
            edge_count[{u,v}]++;
        }
    }

    std::vector<std::pair<int,int>> boundary_edges;
    for (auto& kv : edge_count) {
        if (kv.second == 1) boundary_edges.push_back(kv.first);
    }
    if (boundary_edges.empty()) return;

    // Order them into a polygon
    int start = boundary_edges[0].first;
    int cur = boundary_edges[0].second;
    boundary_verts.push_back(start);
    boundary_verts.push_back(cur);

    std::map<int, std::vector<int>> adj;
    for (auto& e : boundary_edges) {
        adj[e.first].push_back(e.second);
        adj[e.second].push_back(e.first);
    }

    while (cur != start) {
        auto& neighbors = adj[cur];
        int next = -1;
        for (int n : neighbors) {
            if (n != boundary_verts[boundary_verts.size()-2]) { next = n; break; }
        }
        if (next == -1) break;
        boundary_verts.push_back(next);
        cur = next;
    }
}

void triangulate_polygon(const std::vector<int>& verts, std::vector<unsigned int>& out_indices) {
    if (verts.size() < 3) return;
    int first = verts[0];
    for (size_t i = 1; i + 1 < verts.size(); ++i) {
        out_indices.push_back((unsigned int)first);
        out_indices.push_back((unsigned int)verts[i]);
        out_indices.push_back((unsigned int)verts[i+1]);
    }
}

void get_face_plane(int face_idx, float& nx, float& ny, float& nz, float& d) {
    if (face_idx < 0 || face_idx >= (int)g_face_list.size()) { nx=0; ny=0; nz=1; d=0; return; }
    MeshFace& face = g_face_list[face_idx];
    int tri0 = face.tri_indices[0];
    int a = g_indices[tri0*3], b = g_indices[tri0*3+1], c = g_indices[tri0*3+2];
    float ax = g_vertices[a*3], ay = g_vertices[a*3+1], az = g_vertices[a*3+2];
    float bx = g_vertices[b*3], by = g_vertices[b*3+1], bz = g_vertices[b*3+2];
    float cx = g_vertices[c*3], cy = g_vertices[c*3+1], cz = g_vertices[c*3+2];
    float ex1 = bx-ax, ey1 = by-ay, ez1 = bz-az;
    float ex2 = cx-ax, ey2 = cy-ay, ez2 = cz-az;
    nx = ey1*ez2 - ez1*ey2;
    ny = ez1*ex2 - ex1*ez2;
    nz = ex1*ey2 - ey1*ex2;
    float len = sqrtf(nx*nx + ny*ny + nz*nz);
    if (len > 0.0001f) { nx /= len; ny /= len; nz /= len; }
    else { nx = 0; ny = 0; nz = 1; }
    d = -(nx*ax + ny*ay + nz*az);
}

static std::array<float,3> project_onto_plane(const float p[3], float nx, float ny, float nz, float d) {
    float dist = nx*p[0] + ny*p[1] + nz*p[2] + d;
    return {p[0] - dist*nx, p[1] - dist*ny, p[2] - dist*nz};
}

// Clip a polygon (subject) against a single edge of the clip polygon.
// The clip edge is defined by two points (A, B) in 2D (projected coordinates).
// We'll work in 3D but operate on the plane.
static std::vector<std::array<float,3>> clip_polygon_against_edge(
    const std::vector<std::array<float,3>>& subject,
    const std::array<float,3>& A,
    const std::array<float,3>& B,
    float nx, float ny, float nz) // face normal (for inside test)
{
    std::vector<std::array<float,3>> output;
    if (subject.empty()) return output;

    // Compute normal of the clip edge in the face plane: cross(face_normal, (B-A))
    float ex = B[0]-A[0], ey = B[1]-A[1], ez = B[2]-A[2];
    float cnx = ny*ez - nz*ey;
    float cny = nz*ex - nx*ez;
    float cnz = nx*ey - ny*ex;
    // normalize
    float len = sqrtf(cnx*cnx + cny*cny + cnz*cnz);
    if (len < 1e-8f) return subject; // edge too small
    cnx /= len; cny /= len; cnz /= len;

    // Plane constant for clip edge (through A)
    float c_d = -(cnx*A[0] + cny*A[1] + cnz*A[2]);

    // We'll use the signed distance to determine inside/outside.
    // Inside = distance >= 0 (point is on the same side as the polygon interior).
    // For convex polygon, interior is to the left of each edge (A->B).
    // We'll test if a point is inside by checking its distance to the edge plane.
    auto dist_to_edge = [&](const std::array<float,3>& P) -> float {
        return cnx*P[0] + cny*P[1] + cnz*P[2] + c_d;
    };

    int n = subject.size();
    for (int i = 0; i < n; ++i) {
        const auto& S = subject[i];
        const auto& E = subject[(i+1)%n];
        float dS = dist_to_edge(S);
        float dE = dist_to_edge(E);
        if (dS >= 0) {
            output.push_back(S);
        }
        if ((dS > 0 && dE < 0) || (dS < 0 && dE > 0)) {
            // Intersection point
            float t = dS / (dS - dE);
            std::array<float,3> I;
            I[0] = S[0] + t*(E[0]-S[0]);
            I[1] = S[1] + t*(E[1]-S[1]);
            I[2] = S[2] + t*(E[2]-S[2]);
            output.push_back(I);
        }
    }
    return output;
}

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

    rebuild_faces();
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

    // ---- 1. Collect selected face indices ----
    std::vector<int> selected_faces = g_selected_faces;
    
    // ---- 2. Compute average normal ----
    float avg_nx = 0, avg_ny = 0, avg_nz = 0;
    for (int tri : selected_faces) {
        int a = (int)g_indices[tri*3], b = (int)g_indices[tri*3+1], c = (int)g_indices[tri*3+2];
        float ax = g_vertices[a*3], ay = g_vertices[a*3+1], az = g_vertices[a*3+2];
        float bx = g_vertices[b*3], by = g_vertices[b*3+1], bz = g_vertices[b*3+2];
        float cx = g_vertices[c*3], cy = g_vertices[c*3+1], cz = g_vertices[c*3+2];
        float ex1 = bx-ax, ey1 = by-ay, ez1 = bz-az;
        float ex2 = cx-ax, ey2 = cy-ay, ez2 = cz-az;
        float nx = ey1*ez2 - ez1*ey2;
        float ny = ez1*ex2 - ex1*ez2;
        float nz = ex1*ey2 - ey1*ex2;
        float len = sqrtf(nx*nx + ny*ny + nz*nz);
        if (len > 0.0001f) { avg_nx += nx/len; avg_ny += ny/len; avg_nz += nz/len; }
    }
    float alen = sqrtf(avg_nx*avg_nx + avg_ny*avg_ny + avg_nz*avg_nz);
    if (alen > 0.0001f) { avg_nx /= alen; avg_ny /= alen; avg_nz /= alen; }
    else { avg_nz = 1.0f; }

    float offset = 0.5f;

    // ---- 3. Build a map from old vertex index to new vertex index (for each use) ----
    // We'll duplicate vertices for each triangle to get flat shading.
    // First, create a copy of the original mesh data.
    std::vector<float> old_vertices = g_vertices;
    std::vector<unsigned int> old_indices = g_indices;

    // We will append new triangles to the mesh.
    // For each selected face, we need to add:
    //   - a new cap triangle (using new vertices)
    //   - side triangles for each boundary edge (using original and new vertices)

    // But to get flat shading, each new triangle must have its own vertices.
    // So we'll create vertices per triangle.

    // We'll collect all the new vertices in a temporary list, and then append them.
    // We'll also need to keep track of which original edge corresponds to which new edge.

    // For simplicity, we'll just create vertices for each triangle as we go.

    // --- Build edge boundary map ---
    std::map<std::pair<int,int>, int> edge_count;
    for (int tri : selected_faces) {
        int v0 = (int)old_indices[tri*3], v1 = (int)old_indices[tri*3+1], v2 = (int)old_indices[tri*3+2];
        int v[3] = {v0, v1, v2};
        for (int i = 0; i < 3; i++) {
            int a = v[i], b = v[(i+1)%3];
            if (a > b) std::swap(a, b);
            edge_count[{a, b}]++;
        }
    }

    // For each selected face, create new geometry
    for (int tri : selected_faces) {
        int v0 = (int)old_indices[tri*3], v1 = (int)old_indices[tri*3+1], v2 = (int)old_indices[tri*3+2];
        int v[3] = {v0, v1, v2};

        // ---- Cap triangle (new extruded face) ----
        // Create 3 new vertices for the cap
        int new_idx[3];
        for (int i = 0; i < 3; i++) {
            float new_x = old_vertices[v[i]*3] + avg_nx * offset;
            float new_y = old_vertices[v[i]*3+1] + avg_ny * offset;
            float new_z = old_vertices[v[i]*3+2] + avg_nz * offset;
            new_idx[i] = (int)g_vertices.size() / 3;
            g_vertices.push_back(new_x);
            g_vertices.push_back(new_y);
            g_vertices.push_back(new_z);
            g_normals.push_back(0.0f);
            g_normals.push_back(0.0f);
            g_normals.push_back(1.0f);
        }
        // Add cap triangle (order same as original to maintain outward normal)
        g_indices.push_back((unsigned int)new_idx[0]);
        g_indices.push_back((unsigned int)new_idx[1]);
        g_indices.push_back((unsigned int)new_idx[2]);

        // ---- Side quads for each edge ----
        for (int i = 0; i < 3; i++) {
            int a = v[i], b = v[(i+1)%3];
            int a_orig = a, b_orig = b;
            if (a_orig > b_orig) std::swap(a_orig, b_orig);
            if (edge_count[{a_orig, b_orig}] == 1) { // boundary edge
                // Create 4 new vertices: two from original edge, two from new edge
                // We want flat shading, so each side triangle gets its own vertices.
                // For quad (a, b, nb, na) we split into two triangles: (a,b,nb) and (a,nb,na)
                // We'll create vertices for each triangle separately.

                // Triangle 1: (a, b, nb)
                int p0 = (int)g_vertices.size() / 3;
                g_vertices.push_back(old_vertices[a*3]);
                g_vertices.push_back(old_vertices[a*3+1]);
                g_vertices.push_back(old_vertices[a*3+2]);
                g_normals.push_back(0.0f); g_normals.push_back(0.0f); g_normals.push_back(1.0f);

                int p1 = (int)g_vertices.size() / 3;
                g_vertices.push_back(old_vertices[b*3]);
                g_vertices.push_back(old_vertices[b*3+1]);
                g_vertices.push_back(old_vertices[b*3+2]);
                g_normals.push_back(0.0f); g_normals.push_back(0.0f); g_normals.push_back(1.0f);

                int p2 = (int)g_vertices.size() / 3;
                g_vertices.push_back(old_vertices[b*3] + avg_nx * offset);
                g_vertices.push_back(old_vertices[b*3+1] + avg_ny * offset);
                g_vertices.push_back(old_vertices[b*3+2] + avg_nz * offset);
                g_normals.push_back(0.0f); g_normals.push_back(0.0f); g_normals.push_back(1.0f);

                // Triangle 2: (a, nb, na)
                int p3 = (int)g_vertices.size() / 3;
                g_vertices.push_back(old_vertices[a*3]);
                g_vertices.push_back(old_vertices[a*3+1]);
                g_vertices.push_back(old_vertices[a*3+2]);
                g_normals.push_back(0.0f); g_normals.push_back(0.0f); g_normals.push_back(1.0f);

                int p4 = (int)g_vertices.size() / 3;
                g_vertices.push_back(old_vertices[b*3] + avg_nx * offset);
                g_vertices.push_back(old_vertices[b*3+1] + avg_ny * offset);
                g_vertices.push_back(old_vertices[b*3+2] + avg_nz * offset);
                g_normals.push_back(0.0f); g_normals.push_back(0.0f); g_normals.push_back(1.0f);

                int p5 = (int)g_vertices.size() / 3;
                g_vertices.push_back(old_vertices[a*3] + avg_nx * offset);
                g_vertices.push_back(old_vertices[a*3+1] + avg_ny * offset);
                g_vertices.push_back(old_vertices[a*3+2] + avg_nz * offset);
                g_normals.push_back(0.0f); g_normals.push_back(0.0f); g_normals.push_back(1.0f);

                // Add the two triangles (ensure correct winding for outward normals)
                // For triangle 1: (a, b, nb)
                g_indices.push_back((unsigned int)p0);
                g_indices.push_back((unsigned int)p1);
                g_indices.push_back((unsigned int)p2);
                // For triangle 2: (a, nb, na)
                g_indices.push_back((unsigned int)p3);
                g_indices.push_back((unsigned int)p4);
                g_indices.push_back((unsigned int)p5);
            }
        }
    }

    // ---- Clear selection ----
    g_selected_faces.clear();
    g_selected_edges.clear();
    g_selected.clear();

    // ---- Recompute normals (now each triangle has unique vertices, so normals will be per-triangle) ----
    update_mesh_buffers();
    // Since each vertex is used by only one triangle, compute_normals() will set each vertex normal to the face normal.
    compute_normals();
    update_gizmo_selection();

    printf("Extrude complete (flat shading enforced)\n");
}

void weld_vertices(float threshold, bool weld_selected_to_center, bool weld_selected_to_first) {
    if (g_vertices.empty()) return;

    if (weld_selected_to_center || weld_selected_to_first) {
        if (g_selected.size() < 2) return;

        float tx = 0, ty = 0, tz = 0;
        if (weld_selected_to_center) {
            for (int idx : g_selected) {
                tx += g_vertices[idx*3];
                ty += g_vertices[idx*3+1];
                tz += g_vertices[idx*3+2];
            }
            tx /= g_selected.size();
            ty /= g_selected.size();
            tz /= g_selected.size();
        } else {
            int first = g_selected[0];
            tx = g_vertices[first*3];
            ty = g_vertices[first*3+1];
            tz = g_vertices[first*3+2];
        }

        int target_idx = g_selected[0];
        g_vertices[target_idx*3] = tx;
        g_vertices[target_idx*3+1] = ty;
        g_vertices[target_idx*3+2] = tz;

        for (size_t i = 0; i < g_indices.size(); ++i) {
            int v = (int)g_indices[i];
            if (std::find(g_selected.begin(), g_selected.end(), v) != g_selected.end() && v != target_idx) {
                g_indices[i] = (unsigned int)target_idx;
            }
        }

        std::vector<bool> keep(g_vertices.size()/3, true);
        for (int idx : g_selected) {
            if (idx != target_idx) keep[idx] = false;
        }
        std::vector<float> new_verts;
        std::vector<int> remap(g_vertices.size()/3, -1);
        for (size_t i = 0; i < keep.size(); ++i) {
            if (keep[i]) {
                remap[i] = (int)new_verts.size()/3;
                new_verts.push_back(g_vertices[i*3]);
                new_verts.push_back(g_vertices[i*3+1]);
                new_verts.push_back(g_vertices[i*3+2]);
            }
        }
        for (size_t i = 0; i < g_indices.size(); ++i) {
            int old = (int)g_indices[i];
            g_indices[i] = (unsigned int)remap[old];
        }
        g_vertices = new_verts;
        g_selected.clear();
        g_selected_faces.clear();
        g_selected_edges.clear();
        update_mesh_buffers();
        compute_normals();
        rebuild_faces();
        update_gizmo_selection();
        return;
    }

    // ---- Original weld by distance ----
    int old_count = (int)g_vertices.size() / 3;
    std::vector<float> new_verts;
    std::vector<int> remap(g_vertices.size() / 3, -1);

    for (size_t i = 0; i < g_vertices.size() / 3; i++) {
        bool found = false;
        for (size_t j = 0; j < new_verts.size() / 3; j++) {
            float dx = g_vertices[i*3] - new_verts[j*3];
            float dy = g_vertices[i*3+1] - new_verts[j*3+1];
            float dz = g_vertices[i*3+2] - new_verts[j*3+2];
            if (dx*dx + dy*dy + dz*dz < threshold*threshold) {
                remap[i] = (int)j;
                found = true;
                break;
            }
        }
        if (!found) {
            remap[i] = (int)(new_verts.size() / 3);
            new_verts.push_back(g_vertices[i*3]);
            new_verts.push_back(g_vertices[i*3+1]);
            new_verts.push_back(g_vertices[i*3+2]);
        }
    }

    for (size_t i = 0; i < g_indices.size(); i++) {
        g_indices[i] = (unsigned int)remap[(int)g_indices[i]];
    }

    g_vertices = new_verts;
    g_selected_faces.clear();
    g_selected_edges.clear();
    update_mesh_buffers();
    compute_normals();
    rebuild_faces();
    update_gizmo_selection();
    printf("Welded vertices: %d -> %zu\n", old_count, g_vertices.size()/3);
}

// ---- Split Edge (winding-preserving) ----
void split_edge(int v0, int v1, float t) {
    if (t <= 0.0f || t >= 1.0f) return;

    int a = std::min(v0, v1);
    int b = std::max(v0, v1);

    std::map<std::pair<int,int>, std::vector<int>> edge_to_tris;
    for (size_t i = 0; i < g_indices.size() / 3; ++i) {
        int v[3] = { (int)g_indices[i*3], (int)g_indices[i*3+1], (int)g_indices[i*3+2] };
        for (int j = 0; j < 3; ++j) {
            int u = std::min(v[j], v[(j+1)%3]);
            int w = std::max(v[j], v[(j+1)%3]);
            edge_to_tris[{u,w}].push_back((int)i);
        }
    }

    auto it = edge_to_tris.find({a,b});
    if (it == edge_to_tris.end()) return;
    const std::vector<int>& tri_indices = it->second;
    if (tri_indices.empty()) return;

    // Create new vertex
    float new_x = g_vertices[a*3] + t * (g_vertices[b*3] - g_vertices[a*3]);
    float new_y = g_vertices[a*3+1] + t * (g_vertices[b*3+1] - g_vertices[a*3+1]);
    float new_z = g_vertices[a*3+2] + t * (g_vertices[b*3+2] - g_vertices[a*3+2]);
    int new_idx = (int)g_vertices.size() / 3;
    g_vertices.push_back(new_x);
    g_vertices.push_back(new_y);
    g_vertices.push_back(new_z);
    g_normals.push_back(0.0f);
    g_normals.push_back(0.0f);
    g_normals.push_back(1.0f);

    std::vector<unsigned int> new_indices;
    new_indices.reserve(g_indices.size() + tri_indices.size() * 3);

    auto compute_tri_normal = [&](int a, int b, int c) -> std::array<float,3> {
        float ax = g_vertices[a*3], ay = g_vertices[a*3+1], az = g_vertices[a*3+2];
        float bx = g_vertices[b*3], by = g_vertices[b*3+1], bz = g_vertices[b*3+2];
        float cx = g_vertices[c*3], cy = g_vertices[c*3+1], cz = g_vertices[c*3+2];
        float ex1 = bx-ax, ey1 = by-ay, ez1 = bz-az;
        float ex2 = cx-ax, ey2 = cy-ay, ez2 = cz-az;
        float nx = ey1*ez2 - ez1*ey2;
        float ny = ez1*ex2 - ex1*ez2;
        float nz = ex1*ey2 - ey1*ex2;
        float len = sqrtf(nx*nx + ny*ny + nz*nz);
        if (len > 0.0001f) return {nx/len, ny/len, nz/len};
        return {0.0f, 0.0f, 1.0f};
    };

    for (size_t i = 0; i < g_indices.size() / 3; ++i) {
        bool should_split = false;
        for (int idx : tri_indices) {
            if ((int)i == idx) { should_split = true; break; }
        }

        if (should_split) {
            int v[3] = { (int)g_indices[i*3], (int)g_indices[i*3+1], (int)g_indices[i*3+2] };
            int pos_a = -1, pos_b = -1;
            for (int j = 0; j < 3; ++j) {
                if (v[j] == a) pos_a = j;
                if (v[j] == b) pos_b = j;
            }
            int c = -1;
            for (int j = 0; j < 3; ++j) {
                if (v[j] != a && v[j] != b) { c = v[j]; break; }
            }
            if (pos_a == -1 || pos_b == -1 || c == -1) continue;

            std::array<float,3> orig_n = compute_tri_normal(v[0], v[1], v[2]);

            auto add_tri = [&](int a, int b, int c) {
                std::array<float,3> tri_n = compute_tri_normal(a, b, c);
                float dot = tri_n[0]*orig_n[0] + tri_n[1]*orig_n[1] + tri_n[2]*orig_n[2];
                if (dot < 0.0f) std::swap(b, c);
                new_indices.push_back((unsigned int)a);
                new_indices.push_back((unsigned int)b);
                new_indices.push_back((unsigned int)c);
            };

            add_tri(a, new_idx, c);
            add_tri(new_idx, b, c);
        } else {
            new_indices.push_back(g_indices[i*3]);
            new_indices.push_back(g_indices[i*3+1]);
            new_indices.push_back(g_indices[i*3+2]);
        }
    }

    g_indices = new_indices;

    // FIXED: topology changed — clear stale selections and rebuild face groups
    g_selected_faces.clear();
    g_selected_edges.clear();

    update_mesh_buffers();
    compute_normals();
    rebuild_faces();
    update_gizmo_selection();
}

// ---- Knife Cut (winding-preserving, edge-cached) ----
void knife_cut(const float start[3], const float end[3]) {
    extern float cam_theta, cam_phi;
    float vx = -sinf(cam_theta) * cosf(cam_phi);
    float vy = -sinf(cam_phi);
    float vz = -cosf(cam_theta) * cosf(cam_phi);
    float len_v = sqrtf(vx*vx + vy*vy + vz*vz);
    if (len_v < 0.0001f) { vx = 0; vy = 0; vz = 1; len_v = 1; }
    vx /= len_v; vy /= len_v; vz /= len_v;

    float lx = end[0] - start[0];
    float ly = end[1] - start[1];
    float lz = end[2] - start[2];
    float len_l = sqrtf(lx*lx + ly*ly + lz*lz);
    if (len_l < 0.0001f) {
        printf("Knife: start and end are too close - aborting.\n");
        return;
    }

    float nx = ly * vz - lz * vy;
    float ny = lz * vx - lx * vz;
    float nz = lx * vy - ly * vx;
    float len_n = sqrtf(nx*nx + ny*ny + nz*nz);
    if (len_n < 0.0001f) {
        printf("Knife: line is parallel to view direction - aborting.\n");
        return;
    }
    nx /= len_n; ny /= len_n; nz /= len_n;

    float d_plane = -(nx * start[0] + ny * start[1] + nz * start[2]);

    std::map<std::pair<int,int>, int> edge_vert_cache;

    auto get_intersection = [&](int a, int b, float da, float db) -> int {
        if (a > b) { std::swap(a, b); std::swap(da, db); }
        auto key = std::make_pair(a, b);
        auto it = edge_vert_cache.find(key);
        if (it != edge_vert_cache.end()) return it->second;

        float t = da / (da - db);
        float new_x = g_vertices[a*3] + t * (g_vertices[b*3] - g_vertices[a*3]);
        float new_y = g_vertices[a*3+1] + t * (g_vertices[b*3+1] - g_vertices[a*3+1]);
        float new_z = g_vertices[a*3+2] + t * (g_vertices[b*3+2] - g_vertices[a*3+2]);
        int idx = (int)g_vertices.size() / 3;
        g_vertices.push_back(new_x);
        g_vertices.push_back(new_y);
        g_vertices.push_back(new_z);
        g_normals.push_back(0.0f);
        g_normals.push_back(0.0f);
        g_normals.push_back(1.0f);
        edge_vert_cache[key] = idx;
        return idx;
    };

    auto compute_tri_normal = [&](int a, int b, int c) -> std::array<float,3> {
        float ax = g_vertices[a*3], ay = g_vertices[a*3+1], az = g_vertices[a*3+2];
        float bx = g_vertices[b*3], by = g_vertices[b*3+1], bz = g_vertices[b*3+2];
        float cx = g_vertices[c*3], cy = g_vertices[c*3+1], cz = g_vertices[c*3+2];
        float ex1 = bx-ax, ey1 = by-ay, ez1 = bz-az;
        float ex2 = cx-ax, ey2 = cy-ay, ez2 = cz-az;
        float nx = ey1*ez2 - ez1*ey2;
        float ny = ez1*ex2 - ex1*ez2;
        float nz = ex1*ey2 - ey1*ex2;
        float len = sqrtf(nx*nx + ny*ny + nz*nz);
        if (len > 0.0001f) return {nx/len, ny/len, nz/len};
        return {0.0f, 0.0f, 1.0f};
    };

    std::vector<unsigned int> new_indices;
    new_indices.reserve(g_indices.size() * 2);

    const float EPS = 1e-6f;

    for (size_t i = 0; i < g_indices.size() / 3; ++i) {
        int v[3] = { (int)g_indices[i*3], (int)g_indices[i*3+1], (int)g_indices[i*3+2] };
        float d0 = g_vertices[v[0]*3]*nx + g_vertices[v[0]*3+1]*ny + g_vertices[v[0]*3+2]*nz + d_plane;
        float d1 = g_vertices[v[1]*3]*nx + g_vertices[v[1]*3+1]*ny + g_vertices[v[1]*3+2]*nz + d_plane;
        float d2 = g_vertices[v[2]*3]*nx + g_vertices[v[2]*3+1]*ny + g_vertices[v[2]*3+2]*nz + d_plane;

        int neg_count = (d0 < -EPS) + (d1 < -EPS) + (d2 < -EPS);
        int pos_count = (d0 > EPS) + (d1 > EPS) + (d2 > EPS);

        if (pos_count == 0 || neg_count == 0) {
            new_indices.push_back(g_indices[i*3]);
            new_indices.push_back(g_indices[i*3+1]);
            new_indices.push_back(g_indices[i*3+2]);
            continue;
        }

        std::array<float,3> orig_n = compute_tri_normal(v[0], v[1], v[2]);

        float dist[3] = {d0, d1, d2};
        int verts[3] = {v[0], v[1], v[2]};
        std::vector<int> pos_verts, neg_verts;
        // FIXED: bucket using the same EPS tolerance as the count check above,
        // instead of a bare >= 0 test, to avoid sliver triangles near the cut plane.
        for (int j = 0; j < 3; ++j) {
            float dj = (fabsf(dist[j]) <= EPS) ? 0.0f : dist[j];
            if (dj >= 0) pos_verts.push_back(verts[j]);
            else neg_verts.push_back(verts[j]);
        }

        std::map<int, float> dmap;
        dmap[v[0]] = d0;
        dmap[v[1]] = d1;
        dmap[v[2]] = d2;

        auto get_intersection_for_edge = [&](int a, int b) -> int {
            float da = dmap[a], db = dmap[b];
            return get_intersection(a, b, da, db);
        };

        auto add_tri = [&](int a, int b, int c) {
            std::array<float,3> tri_n = compute_tri_normal(a, b, c);
            float dot = tri_n[0]*orig_n[0] + tri_n[1]*orig_n[1] + tri_n[2]*orig_n[2];
            if (dot < 0.0f) std::swap(b, c);
            new_indices.push_back((unsigned int)a);
            new_indices.push_back((unsigned int)b);
            new_indices.push_back((unsigned int)c);
        };

        if (pos_verts.size() == 1 && neg_verts.size() == 2) {
            int p = pos_verts[0];
            int n1 = neg_verts[0];
            int n2 = neg_verts[1];
            int i1 = get_intersection_for_edge(p, n1);
            int i2 = get_intersection_for_edge(p, n2);
            add_tri(p, i1, i2);
            add_tri(i1, n1, n2);
            add_tri(i1, n2, i2);
        } else if (pos_verts.size() == 2 && neg_verts.size() == 1) {
            int p1 = pos_verts[0], p2 = pos_verts[1];
            int n = neg_verts[0];
            int i1 = get_intersection_for_edge(p1, n);
            int i2 = get_intersection_for_edge(p2, n);
            add_tri(p1, p2, i1);
            add_tri(p2, i2, i1);
            add_tri(i1, i2, n);
        } else {
            // Degenerate case after EPS re-bucketing (e.g. all on one side) — keep original tri
            new_indices.push_back((unsigned int)v[0]);
            new_indices.push_back((unsigned int)v[1]);
            new_indices.push_back((unsigned int)v[2]);
        }
    }

    g_indices = new_indices;

    // FIXED: topology fully changed — clear stale selection state and rebuild
    // face groups so face-select mode doesn't reference the wrong triangles.
    g_selected.clear();
    g_selected_faces.clear();
    g_selected_edges.clear();

    update_mesh_buffers();
    compute_normals();
    rebuild_faces();
    update_gizmo_selection();
    printf("Knife cut performed (cached edges, winding preserved)\n");
}

// ---- Unify Normals (flips faces to make normals consistent) ----
void unify_normals() {
    if (g_indices.empty()) return;

    int num_tris = (int)g_indices.size() / 3;
    std::vector<bool> visited(num_tris, false);
    std::vector<std::array<float,3>> tri_normals(num_tris);

    for (int i = 0; i < num_tris; ++i) {
        int a = g_indices[i*3], b = g_indices[i*3+1], c = g_indices[i*3+2];
        float ax = g_vertices[a*3], ay = g_vertices[a*3+1], az = g_vertices[a*3+2];
        float bx = g_vertices[b*3], by = g_vertices[b*3+1], bz = g_vertices[b*3+2];
        float cx = g_vertices[c*3], cy = g_vertices[c*3+1], cz = g_vertices[c*3+2];
        float ex1 = bx-ax, ey1 = by-ay, ez1 = bz-az;
        float ex2 = cx-ax, ey2 = cy-ay, ez2 = cz-az;
        float nx = ey1*ez2 - ez1*ey2;
        float ny = ez1*ex2 - ex1*ez2;
        float nz = ex1*ey2 - ey1*ex2;
        float len = sqrtf(nx*nx + ny*ny + nz*nz);
        if (len > 0.0001f) tri_normals[i] = {nx/len, ny/len, nz/len};
        else tri_normals[i] = {0,0,1};
    }

    std::map<std::pair<int,int>, std::vector<int>> edge_to_tris;
    for (int i = 0; i < num_tris; ++i) {
        int a = g_indices[i*3], b = g_indices[i*3+1], c = g_indices[i*3+2];
        int edges[3][2] = {{a,b}, {b,c}, {c,a}};
        for (int j = 0; j < 3; ++j) {
            int u = std::min(edges[j][0], edges[j][1]);
            int v = std::max(edges[j][0], edges[j][1]);
            edge_to_tris[{u,v}].push_back(i);
        }
    }

    int flip_count = 0;
    for (int start = 0; start < num_tris; ++start) {
        if (visited[start]) continue;

        std::queue<int> q;
        q.push(start);
        visited[start] = true;

        while (!q.empty()) {
            int t = q.front();
            q.pop();

            int a = g_indices[t*3], b = g_indices[t*3+1], c = g_indices[t*3+2];
            int edges[3][2] = {{a,b}, {b,c}, {c,a}};

            for (int j = 0; j < 3; ++j) {
                int u = std::min(edges[j][0], edges[j][1]);
                int v = std::max(edges[j][0], edges[j][1]);
                auto it = edge_to_tris.find({u,v});
                if (it == edge_to_tris.end()) continue;

                for (int neighbor : it->second) {
                    if (visited[neighbor]) continue;

                    float dot = tri_normals[t][0]*tri_normals[neighbor][0] +
                                tri_normals[t][1]*tri_normals[neighbor][1] +
                                tri_normals[t][2]*tri_normals[neighbor][2];

                    if (dot < 0.0f) {
                        int idx0 = neighbor*3, idx1 = neighbor*3+1, idx2 = neighbor*3+2;
                        std::swap(g_indices[idx1], g_indices[idx2]);
                        tri_normals[neighbor] = {-tri_normals[neighbor][0],
                                                  -tri_normals[neighbor][1],
                                                  -tri_normals[neighbor][2]};
                        flip_count++;
                    }

                    visited[neighbor] = true;
                    q.push(neighbor);
                }
            }
        }
    }

    update_mesh_buffers();
    compute_normals();
    printf("Unified normals: flipped %d faces\n", flip_count);
}

// ---- Rebuild Faces (already existing, included for completeness) ----
void rebuild_faces() {
    g_face_list.clear();
    if (g_indices.empty()) return;
    int num_tris = (int)g_indices.size() / 3;
    if (num_tris == 0) return;

    std::vector<bool> visited(num_tris, false);
    std::map<std::pair<int,int>, std::vector<int>> edge_to_tris;
    for (int i = 0; i < num_tris; i++) {
        int a = (int)g_indices[i*3];
        int b = (int)g_indices[i*3+1];
        int c = (int)g_indices[i*3+2];
        int edges[3][2] = {{a,b}, {b,c}, {c,a}};
        for (int j = 0; j < 3; j++) {
            int u = std::min(edges[j][0], edges[j][1]);
            int v = std::max(edges[j][0], edges[j][1]);
            edge_to_tris[{u,v}].push_back(i);
        }
    }

    std::vector<std::array<float,3>> tri_normals(num_tris);
    for (int i = 0; i < num_tris; i++) {
        int a = (int)g_indices[i*3];
        int b = (int)g_indices[i*3+1];
        int c = (int)g_indices[i*3+2];
        float ax = g_vertices[a*3], ay = g_vertices[a*3+1], az = g_vertices[a*3+2];
        float bx = g_vertices[b*3], by = g_vertices[b*3+1], bz = g_vertices[b*3+2];
        float cx = g_vertices[c*3], cy = g_vertices[c*3+1], cz = g_vertices[c*3+2];
        float ex1 = bx-ax, ey1 = by-ay, ez1 = bz-az;
        float ex2 = cx-ax, ey2 = cy-ay, ez2 = cz-az;
        float nx = ey1*ez2 - ez1*ey2;
        float ny = ez1*ex2 - ex1*ez2;
        float nz = ex1*ey2 - ey1*ex2;
        float len = sqrtf(nx*nx + ny*ny + nz*nz);
        if (len > 0.0001f) tri_normals[i] = {nx/len, ny/len, nz/len};
        else tri_normals[i] = {0,0,1};
    }

    const float COPLANAR_THRESHOLD = 0.99f;
    for (int i = 0; i < num_tris; i++) {
        if (visited[i]) continue;
        std::vector<int> tri_group;
        std::queue<int> q;
        q.push(i);
        visited[i] = true;
        while (!q.empty()) {
            int t = q.front();
            q.pop();
            tri_group.push_back(t);
            int a = (int)g_indices[t*3];
            int b = (int)g_indices[t*3+1];
            int c = (int)g_indices[t*3+2];
            int edges[3][2] = {{a,b}, {b,c}, {c,a}};
            for (int j = 0; j < 3; j++) {
                int u = std::min(edges[j][0], edges[j][1]);
                int v = std::max(edges[j][0], edges[j][1]);
                auto it = edge_to_tris.find({u,v});
                if (it == edge_to_tris.end()) continue;
                for (int neighbor : it->second) {
                    if (visited[neighbor]) continue;
                    float dot = tri_normals[t][0]*tri_normals[neighbor][0] +
                                tri_normals[t][1]*tri_normals[neighbor][1] +
                                tri_normals[t][2]*tri_normals[neighbor][2];
                    if (dot > COPLANAR_THRESHOLD) {
                        visited[neighbor] = true;
                        q.push(neighbor);
                    }
                }
            }
        }
        if (!tri_group.empty()) {
            MeshFace face;
            face.tri_indices = tri_group;
            g_face_list.push_back(face);
        }
    }
}

// ---- Make Face from selected vertices (fan triangulation) ----
void make_face_from_selected() {
    if (g_selected.size() < 3) {
        printf("Need at least 3 vertices selected.\n");
        return;
    }

    // Compute centroid
    float cx = 0.0f, cy = 0.0f, cz = 0.0f;
    for (int idx : g_selected) {
        cx += g_vertices[idx*3];
        cy += g_vertices[idx*3+1];
        cz += g_vertices[idx*3+2];
    }
    cx /= g_selected.size();
    cy /= g_selected.size();
    cz /= g_selected.size();

    // Sort vertices by angle around centroid (projected onto XY plane)
    // This works well for convex polygons; for concave it may produce some overlapping,
    // but it's a reasonable low‑poly approximation.
    std::sort(g_selected.begin(), g_selected.end(),
        [&](int a, int b) {
            float ax = g_vertices[a*3] - cx;
            float ay = g_vertices[a*3+1] - cy;
            float bx = g_vertices[b*3] - cx;
            float by = g_vertices[b*3+1] - cy;
            return atan2(ay, ax) < atan2(by, bx);
        });

    // Fan triangulation: (v0, v1, v2), (v0, v2, v3), ...
    int first = g_selected[0];
    for (size_t i = 1; i + 1 < g_selected.size(); i++) {
        add_triangle(first, g_selected[i], g_selected[i+1]);
    }

    // Clear selection (optional – you may want to keep them selected)
    g_selected.clear();

    update_mesh_buffers();
    printf("Created face from %zu vertices.\n", g_selected.size());
}

void split_face_by_polygon(int face_idx, const std::vector<std::array<float,3>>& points) {
    if (face_idx < 0 || face_idx >= (int)g_face_list.size()) return;
    if (points.size() < 3) {
        printf("Need at least 3 points.\n");
        return;
    }

    // ---- 1. Get face plane ----
    float nx, ny, nz, d;
    get_face_plane(face_idx, nx, ny, nz, d);

    // ---- 2. Project the drawn points onto the face plane ----
    std::vector<std::array<float,3>> clip_poly;
    clip_poly.reserve(points.size());
    for (const auto& p : points) {
        std::array<float,3> proj = project_onto_plane(p.data(), nx, ny, nz, d);
        clip_poly.push_back(proj);
    }

    // ---- 3. Get the boundary vertices of the original face ----
    std::vector<int> face_verts;
    get_face_boundary(face_idx, face_verts);
    if (face_verts.size() < 3) {
        printf("Face boundary has less than 3 vertices.\n");
        return;
    }

    // ---- 4. Create 3D coordinates for the face boundary ----
    std::vector<std::array<float,3>> face_poly;
    for (int v : face_verts) {
        face_poly.push_back({g_vertices[v*3], g_vertices[v*3+1], g_vertices[v*3+2]});
    }

    // ---- 5. Clip the face polygon against the drawn polygon (clip_poly) ----
    // We'll clip the face polygon by each edge of the clip polygon.
    std::vector<std::array<float,3>> outer_poly = face_poly;
    for (size_t i = 0; i < clip_poly.size(); ++i) {
        const auto& A = clip_poly[i];
        const auto& B = clip_poly[(i+1)%clip_poly.size()];
        outer_poly = clip_polygon_against_edge(outer_poly, A, B, nx, ny, nz);
        if (outer_poly.size() < 3) break;
    }

    // ---- 6. Now we have two polygons: clip_poly (the drawn one) and outer_poly (the rest) ----
    // We need to create new vertices for the clip polygon if they are not already in the mesh.
    // We'll create new vertices for all points of clip_poly and outer_poly (except those that coincide with existing vertices).
    // For simplicity, we'll create new vertices for each point of both polygons (no sharing).
    // This may produce many vertices but is simple.

    // ---- 7. Build a new index list ----
    // First, remove the original face's triangles.
    std::set<int> face_tri_set;
    for (int tri : g_face_list[face_idx].tri_indices) face_tri_set.insert(tri);

    std::vector<unsigned int> new_indices;
    // Copy all triangles not belonging to this face
    for (size_t i = 0; i < g_indices.size() / 3; ++i) {
        if (face_tri_set.find((int)i) == face_tri_set.end()) {
            new_indices.push_back(g_indices[i*3]);
            new_indices.push_back(g_indices[i*3+1]);
            new_indices.push_back(g_indices[i*3+2]);
        }
    }

    // ---- 8. Create vertices and triangles for the drawn polygon (clip_poly) ----
    std::vector<int> clip_vert_indices;
    for (auto& p : clip_poly) {
        clip_vert_indices.push_back((int)g_vertices.size()/3);
        g_vertices.push_back(p[0]); g_vertices.push_back(p[1]); g_vertices.push_back(p[2]);
        g_normals.push_back(0.0f); g_normals.push_back(0.0f); g_normals.push_back(1.0f);
    }
    // Triangulate clip polygon
    std::vector<unsigned int> clip_tris;
    triangulate_polygon(clip_vert_indices, clip_tris);
    // Add triangles (winding will be corrected later)
    for (auto idx : clip_tris) new_indices.push_back(idx);

    // ---- 9. Create vertices and triangles for the outer polygon ----
    std::vector<int> outer_vert_indices;
    for (auto& p : outer_poly) {
        outer_vert_indices.push_back((int)g_vertices.size()/3);
        g_vertices.push_back(p[0]); g_vertices.push_back(p[1]); g_vertices.push_back(p[2]);
        g_normals.push_back(0.0f); g_normals.push_back(0.0f); g_normals.push_back(1.0f);
    }
    // Triangulate outer polygon
    std::vector<unsigned int> outer_tris;
    triangulate_polygon(outer_vert_indices, outer_tris);
    for (auto idx : outer_tris) new_indices.push_back(idx);

    // ---- 10. Replace indices ----
    g_indices = new_indices;

    // ---- 11. Clear selections and rebuild ----
    g_selected.clear();
    g_selected_faces.clear();
    g_selected_edges.clear();

    update_mesh_buffers();   // rebuilds face groups
    compute_normals();
    update_gizmo_selection();

    printf("Face %d split into two faces (drawn polygon and remainder).\n", face_idx);
}