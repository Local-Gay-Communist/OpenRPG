// viewport.cpp
#include "viewport.h"
#include "globals.h"
#include "math.h"
#include "shader.h"
#include "fbo.h"
#include "mesh.h"
#include "grid.h"
#include "undo.h"
#include <GL/glew.h>
#include <cmath>
#include <algorithm>
#include <queue>
#include "imgui/imgui.h"

// ---- Helper: ground intersection ----
static bool get_ground_intersection(float ndc_x, float ndc_y, float& out_x, float& out_y,
                                    float cam_ex, float cam_ey, float cam_ez,
                                    float aspect) {
    float fov = 3.14159f / 2.0f;
    float near = 0.1f;
    float tanHalf = tanf(fov / 2.0f);
    float A = 1.0f / (aspect * tanHalf);
    float B = 1.0f / tanHalf;

    float x_view = ndc_x * near / A;
    float y_view = ndc_y * near / B;

    float fx = -cam_ex, fy = -cam_ey, fz = -cam_ez;
    float flen = sqrtf(fx*fx + fy*fy + fz*fz);
    if (flen < 0.0001f) return false;
    fx /= flen; fy /= flen; fz /= flen;

    float ux = 0, uy = 1, uz = 0;
    float rx = uy*fz - uz*fy;
    float ry = uz*fx - ux*fz;
    float rz = ux*fy - uy*fx;
    float rlen = sqrtf(rx*rx + ry*ry + rz*rz);
    if (rlen < 0.0001f) { rx = 1; ry = 0; rz = 0; }
    else { rx /= rlen; ry /= rlen; rz /= rlen; }
    float ux2 = fy*rz - fz*ry;
    float uy2 = fz*rx - fx*rz;
    float uz2 = fx*ry - fy*rx;

    float wpx = cam_ex + x_view*rx + y_view*ux2 + near*fx;
    float wpy = cam_ey + x_view*ry + y_view*uy2 + near*fy;
    float wpz = cam_ez + x_view*rz + y_view*uz2 + near*fz;

    float dx = wpx - cam_ex;
    float dy = wpy - cam_ey;
    float dz = wpz - cam_ez;
    if (fabs(dz) < 0.0001f) return false;
    float t = (0.0f - cam_ez) / dz;
    if (t < 0) return false;
    out_x = cam_ex + t * dx;
    out_y = cam_ey + t * dy;
    return true;
}

// ---- Helper: raycast vertex ----
static int raycast_vertex(float ray_ox, float ray_oy, float ray_oz,
                          float ray_dx, float ray_dy, float ray_dz,
                          float threshold_sq) {
    int best = -1; float best_d = threshold_sq;
    for (int i=0; i<g_vert_count; ++i) {
        float vx = g_vertices[i*3], vy = g_vertices[i*3+1], vz = g_vertices[i*3+2];
        float wx = vx - ray_ox, wy = vy - ray_oy, wz = vz - ray_oz;
        float t = wx*ray_dx + wy*ray_dy + wz*ray_dz;
        if (t < 0) t = 0;
        float cx = ray_ox + t*ray_dx, cy = ray_oy + t*ray_dy, cz = ray_oz + t*ray_dz;
        float dx = vx-cx, dy = vy-cy, dz = vz-cz;
        float d2 = dx*dx + dy*dy + dz*dz;
        if (d2 < best_d) { best_d = d2; best = i; }
    }
    return best;
}

// ---- Ray-Triangle Intersection ----
static bool ray_triangle_intersect(const float* orig, const float* dir,
                                   const float* v0, const float* v1, const float* v2,
                                   float& t, float& u, float& v) {
    float edge1[3] = {v1[0]-v0[0], v1[1]-v0[1], v1[2]-v0[2]};
    float edge2[3] = {v2[0]-v0[0], v2[1]-v0[1], v2[2]-v0[2]};
    float pvec[3];
    pvec[0] = dir[1]*edge2[2] - dir[2]*edge2[1];
    pvec[1] = dir[2]*edge2[0] - dir[0]*edge2[2];
    pvec[2] = dir[0]*edge2[1] - dir[1]*edge2[0];
    float det = edge1[0]*pvec[0] + edge1[1]*pvec[1] + edge1[2]*pvec[2];
    if (fabs(det) < 1e-8f) return false;
    float inv_det = 1.0f / det;
    float tvec[3] = {orig[0]-v0[0], orig[1]-v0[1], orig[2]-v0[2]};
    u = (tvec[0]*pvec[0] + tvec[1]*pvec[1] + tvec[2]*pvec[2]) * inv_det;
    if (u < 0 || u > 1) return false;
    float qvec[3];
    qvec[0] = tvec[1]*edge1[2] - tvec[2]*edge1[1];
    qvec[1] = tvec[2]*edge1[0] - tvec[0]*edge1[2];
    qvec[2] = tvec[0]*edge1[1] - tvec[1]*edge1[0];
    v = (dir[0]*qvec[0] + dir[1]*qvec[1] + dir[2]*qvec[2]) * inv_det;
    if (v < 0 || u + v > 1) return false;
    t = (edge2[0]*qvec[0] + edge2[1]*qvec[1] + edge2[2]*qvec[2]) * inv_det;
    return true;
}

// ---- Pick Face ----
static int pick_face(const float* orig, const float* dir, float& out_u, float& out_v) {
    int best_tri = -1;
    float best_t = 1e9f;
    for (size_t i = 0; i < g_indices.size() / 3; i++) {
        int a = (int)g_indices[i*3];
        int b = (int)g_indices[i*3+1];
        int c = (int)g_indices[i*3+2];
        float t, u, v;
        if (ray_triangle_intersect(orig, dir,
                                   &g_vertices[a*3], &g_vertices[b*3], &g_vertices[c*3],
                                   t, u, v)) {
            if (t > 0 && t < best_t) {
                best_t = t;
                best_tri = (int)i;
                out_u = u;
                out_v = v;
            }
        }
    }
    return best_tri;
}

// ---- Pick Edge ----
static std::pair<int,int> pick_edge(const float* orig, const float* dir, float threshold) {
    float u, v;
    int tri_idx = pick_face(orig, dir, u, v);
    if (tri_idx == -1) return {-1, -1};
    
    int a = (int)g_indices[tri_idx*3];
    int b = (int)g_indices[tri_idx*3+1];
    int c = (int)g_indices[tri_idx*3+2];
    
    float hit[3];
    hit[0] = (1-u-v)*g_vertices[a*3] + u*g_vertices[b*3] + v*g_vertices[c*3];
    hit[1] = (1-u-v)*g_vertices[a*3+1] + u*g_vertices[b*3+1] + v*g_vertices[c*3+1];
    hit[2] = (1-u-v)*g_vertices[a*3+2] + u*g_vertices[b*3+2] + v*g_vertices[c*3+2];
    
    std::pair<int,int> edges[3] = {{a,b}, {b,c}, {c,a}};
    float best_dist = threshold;
    std::pair<int,int> best_edge = {-1, -1};
    
    for (auto& edge : edges) {
        float v0[3] = {g_vertices[edge.first*3],   g_vertices[edge.first*3+1],   g_vertices[edge.first*3+2]};
        float v1[3] = {g_vertices[edge.second*3],  g_vertices[edge.second*3+1],  g_vertices[edge.second*3+2]};
        float edge_vec[3] = {v1[0]-v0[0], v1[1]-v0[1], v1[2]-v0[2]};
        float hit_vec[3] = {hit[0]-v0[0], hit[1]-v0[1], hit[2]-v0[2]};
        float t_proj = (hit_vec[0]*edge_vec[0] + hit_vec[1]*edge_vec[1] + hit_vec[2]*edge_vec[2]) /
                       (edge_vec[0]*edge_vec[0] + edge_vec[1]*edge_vec[1] + edge_vec[2]*edge_vec[2]);
        t_proj = std::max(0.0f, std::min(1.0f, t_proj));
        float closest[3] = {v0[0] + t_proj*edge_vec[0], v0[1] + t_proj*edge_vec[1], v0[2] + t_proj*edge_vec[2]};
        float dx = hit[0]-closest[0], dy = hit[1]-closest[1], dz = hit[2]-closest[2];
        float dist = sqrtf(dx*dx + dy*dy + dz*dz);
        if (dist < best_dist) {
            best_dist = dist;
            best_edge = edge;
        }
    }
    return best_edge;
}

// ---- Gizmo functions ----

void update_gizmo_selection() {
    g_gizmo_verts.clear();
    
    if (g_sel_mode == SELECT_VERTEX) {
        for (int v : g_selected) g_gizmo_verts.push_back(v);
    } else if (g_sel_mode == SELECT_EDGE) {
        for (auto& edge : g_selected_edges) {
            g_gizmo_verts.push_back(edge.first);
            g_gizmo_verts.push_back(edge.second);
        }
    } else if (g_sel_mode == SELECT_FACE) {
        for (int tri : g_selected_faces) {
            g_gizmo_verts.push_back((int)g_indices[tri*3]);
            g_gizmo_verts.push_back((int)g_indices[tri*3+1]);
            g_gizmo_verts.push_back((int)g_indices[tri*3+2]);
        }
    }
    
    std::sort(g_gizmo_verts.begin(), g_gizmo_verts.end());
    g_gizmo_verts.erase(std::unique(g_gizmo_verts.begin(), g_gizmo_verts.end()), g_gizmo_verts.end());
    
    if (g_gizmo_verts.empty()) {
        g_gizmo_pos[0] = g_gizmo_pos[1] = g_gizmo_pos[2] = 0.0f;
        return;
    }
    float cx=0, cy=0, cz=0;
    for (int v : g_gizmo_verts) {
        cx += g_vertices[v*3];
        cy += g_vertices[v*3+1];
        cz += g_vertices[v*3+2];
    }
    g_gizmo_pos[0] = cx / g_gizmo_verts.size();
    g_gizmo_pos[1] = cy / g_gizmo_verts.size();
    g_gizmo_pos[2] = cz / g_gizmo_verts.size();
}

static void draw_arrow(const float* origin, const float* dir, float length, float radius, float r, float g, float b, bool highlight) {
    float end[3] = {origin[0] + dir[0]*length, origin[1] + dir[1]*length, origin[2] + dir[2]*length};
    float brightness = highlight ? 1.5f : 1.0f;
    
    glLineWidth(highlight ? 4.0f : 2.5f);
    glBegin(GL_LINES);
    glColor3f(r * brightness, g * brightness, b * brightness);
    glVertex3f(origin[0], origin[1], origin[2]);
    glVertex3f(end[0], end[1], end[2]);
    glEnd();
    
    float cone_len = length * 0.2f;
    float cone_radius = radius * 2.5f;
    float base[3] = {end[0] - dir[0]*cone_len, end[1] - dir[1]*cone_len, end[2] - dir[2]*cone_len};
    
    float up[3] = {0,1,0};
    if (fabs(dir[1]) > 0.99f) up[0] = 1;
    float right[3], fwd[3];
    right[0] = up[1]*dir[2] - up[2]*dir[1];
    right[1] = up[2]*dir[0] - up[0]*dir[2];
    right[2] = up[0]*dir[1] - up[1]*dir[0];
    float rlen = sqrtf(right[0]*right[0] + right[1]*right[1] + right[2]*right[2]);
    if (rlen < 0.0001f) { right[0] = 1; right[1] = 0; right[2] = 0; }
    else { right[0]/=rlen; right[1]/=rlen; right[2]/=rlen; }
    fwd[0] = dir[1]*right[2] - dir[2]*right[1];
    fwd[1] = dir[2]*right[0] - dir[0]*right[2];
    fwd[2] = dir[0]*right[1] - dir[1]*right[0];
    
    glBegin(GL_TRIANGLES);
    glColor3f(r * brightness, g * brightness, b * brightness);
    int segments = 6;
    for (int i = 0; i < segments; i++) {
        float angle1 = (float)i / segments * 2.0f * 3.14159f;
        float angle2 = (float)(i+1) / segments * 2.0f * 3.14159f;
        float c1 = cosf(angle1), s1 = sinf(angle1);
        float c2 = cosf(angle2), s2 = sinf(angle2);
        float p1[3] = {base[0] + cone_radius*(c1*right[0] + s1*fwd[0]),
                       base[1] + cone_radius*(c1*right[1] + s1*fwd[1]),
                       base[2] + cone_radius*(c1*right[2] + s1*fwd[2])};
        float p2[3] = {base[0] + cone_radius*(c2*right[0] + s2*fwd[0]),
                       base[1] + cone_radius*(c2*right[1] + s2*fwd[1]),
                       base[2] + cone_radius*(c2*right[2] + s2*fwd[2])};
        glVertex3f(end[0], end[1], end[2]);
        glVertex3f(p1[0], p1[1], p1[2]);
        glVertex3f(p2[0], p2[1], p2[2]);
    }
    glEnd();
}

static void render_gizmo(const Mat4& view, const Mat4& proj) {
    if (!g_gizmo_enabled || g_gizmo_verts.empty()) return;
    
    glUseProgram(0);
    
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadMatrixf(proj.m);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadMatrixf(view.m);
    
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    
    float origin[3] = {g_gizmo_pos[0], g_gizmo_pos[1], g_gizmo_pos[2]};
    float size = g_gizmo_size;
    float radius = size * 0.03f;
    
    float axes[3][3] = {{1,0,0}, {0,1,0}, {0,0,1}};
    float colors[3][3] = {{1,0.2f,0.2f}, {0.2f,1,0.2f}, {0.2f,0.2f,1}};
    
    for (int i = 0; i < 3; i++) {
        bool highlight = (g_gizmo_axis == i);
        draw_arrow(origin, axes[i], size, radius, colors[i][0], colors[i][1], colors[i][2], highlight);
    }
    
    glPointSize(8.0f);
    glBegin(GL_POINTS);
    glColor3f(1,1,1);
    glVertex3f(origin[0], origin[1], origin[2]);
    glEnd();
    
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
    
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
    
    glUseProgram(shaderProgram);
}

static int pick_gizmo(const float* orig, const float* dir) {
    if (g_gizmo_verts.empty()) return -1;
    float threshold = 0.12f;
    int best_axis = -1;
    float best_dist = threshold;
    
    float axes[3][3] = {{1,0,0}, {0,1,0}, {0,0,1}};
    float origin[3] = {g_gizmo_pos[0], g_gizmo_pos[1], g_gizmo_pos[2]};
    
    for (int i = 0; i < 3; i++) {
        float end[3] = {origin[0] + axes[i][0]*g_gizmo_size,
                        origin[1] + axes[i][1]*g_gizmo_size,
                        origin[2] + axes[i][2]*g_gizmo_size};
        
        float ray_dir[3] = {dir[0], dir[1], dir[2]};
        float seg_dir[3] = {end[0]-origin[0], end[1]-origin[1], end[2]-origin[2]};
        float seg_len = sqrtf(seg_dir[0]*seg_dir[0] + seg_dir[1]*seg_dir[1] + seg_dir[2]*seg_dir[2]);
        if (seg_len < 0.0001f) continue;
        seg_dir[0]/=seg_len; seg_dir[1]/=seg_len; seg_dir[2]/=seg_len;
        
        float w0[3] = {orig[0]-origin[0], orig[1]-origin[1], orig[2]-origin[2]};
        float a = ray_dir[0]*ray_dir[0] + ray_dir[1]*ray_dir[1] + ray_dir[2]*ray_dir[2];
        float b = ray_dir[0]*seg_dir[0] + ray_dir[1]*seg_dir[1] + ray_dir[2]*seg_dir[2];
        float c = seg_dir[0]*seg_dir[0] + seg_dir[1]*seg_dir[1] + seg_dir[2]*seg_dir[2];
        float d = ray_dir[0]*w0[0] + ray_dir[1]*w0[1] + ray_dir[2]*w0[2];
        float e = seg_dir[0]*w0[0] + seg_dir[1]*w0[1] + seg_dir[2]*w0[2];
        float denom = a*c - b*b;
        if (fabs(denom) < 0.0001f) continue;
        float t_ray = (b*e - c*d) / denom;
        float t_seg = (a*e - b*d) / denom;
        if (t_seg < 0 || t_seg > seg_len) continue;
        if (t_ray < 0) continue;
        
        float cp_ray[3] = {orig[0] + t_ray*ray_dir[0],
                           orig[1] + t_ray*ray_dir[1],
                           orig[2] + t_ray*ray_dir[2]};
        float cp_seg[3] = {origin[0] + t_seg*seg_dir[0],
                           origin[1] + t_seg*seg_dir[1],
                           origin[2] + t_seg*seg_dir[2]};
        float dx = cp_ray[0]-cp_seg[0], dy = cp_ray[1]-cp_seg[1], dz = cp_ray[2]-cp_seg[2];
        float dist = sqrtf(dx*dx + dy*dy + dz*dz);
        if (dist < best_dist) {
            best_dist = dist;
            best_axis = i;
        }
    }
    return best_axis;
}

// ---- Screen-space gizmo helpers ----
static void project_point_to_ndc(const float* world, float* ndc, const Mat4& view, const Mat4& proj, int w, int h) {
    Mat4 mvp = mat4_mul(proj, view);
    float x = world[0], y = world[1], z = world[2];
    float tx = mvp.m[0]*x + mvp.m[4]*y + mvp.m[8]*z + mvp.m[12];
    float ty = mvp.m[1]*x + mvp.m[5]*y + mvp.m[9]*z + mvp.m[13];
    float tz = mvp.m[2]*x + mvp.m[6]*y + mvp.m[10]*z + mvp.m[14];
    float tw = mvp.m[3]*x + mvp.m[7]*y + mvp.m[11]*z + mvp.m[15];
    if (tw != 0.0f) { ndc[0] = tx/tw; ndc[1] = ty/tw; ndc[2] = tz/tw; }
    else { ndc[0] = ndc[1] = ndc[2] = 0.0f; }
}

// ---- Main viewport window ----
void RenderViewportWindow() {
    ImGuiViewport* main_vp = ImGui::GetMainViewport();
    ImVec2 work_pos = main_vp->WorkPos;
    ImVec2 work_size = main_vp->WorkSize;

    ImGui::SetNextWindowPos(ImVec2(work_pos.x + work_size.x * 0.20f, work_pos.y + 0), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(work_size.x * 0.55f, work_size.y * 0.80f), ImGuiCond_FirstUseEver);
    ImGui::Begin("Render Viewport", nullptr,
        ImGuiWindowFlags_NoScrollbar |
        ImGuiWindowFlags_NoScrollWithMouse |
        ImGuiWindowFlags_NoMove);

    ImVec2 vp_size = ImGui::GetContentRegionAvail();
    if (vp_size.x > 0 && vp_size.y > 0) {
        if (viewport_width != (int)vp_size.x || viewport_height != (int)vp_size.y) {
            init_fbo((int)vp_size.x, (int)vp_size.y);
        }

        float eye_x = cam_dist * cosf(cam_phi) * sinf(cam_theta);
        float eye_y = cam_dist * sinf(cam_phi);
        float eye_z = cam_dist * cosf(cam_phi) * cosf(cam_theta);
        if (cam_phi > 1.5f) cam_phi = 1.5f;
        if (cam_phi < -1.5f) cam_phi = -1.5f;

        Mat4 view = look_at(eye_x, eye_y, eye_z, 0,0,0, 0,1,0);

        glBindFramebuffer(GL_FRAMEBUFFER, fbo);
        glViewport(0, 0, viewport_width, viewport_height);
        glEnable(GL_DEPTH_TEST);
        glClearColor(0.12f, 0.12f, 0.16f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        Mat4 model = mat4_mul(rotate_z(0.0f), mat4_mul(rotate_y(0.0f), rotate_x(0.0f)));
        float aspect = vp_size.x / vp_size.y;
        Mat4 proj = perspective(3.14159f / 2.0f, aspect, 0.1f, 10.0f);
        Mat4 mv = mat4_mul(view, model);
        Mat4 mvp = mat4_mul(proj, mv);

        render_grid(mvp);

        glUseProgram(shaderProgram);
        int mvpLoc = glGetUniformLocation(shaderProgram, "mvp");
        if (mvpLoc != -1) glUniformMatrix4fv(mvpLoc, 1, GL_FALSE, mvp.m);
        int modelLoc = glGetUniformLocation(shaderProgram, "model");
        if (modelLoc != -1) glUniformMatrix4fv(modelLoc, 1, GL_FALSE, model.m);
        int lightDirLoc = glGetUniformLocation(shaderProgram, "lightDir");
        if (lightDirLoc != -1) {
            float lx = cosf(light_elevation) * sinf(light_azimuth);
            float ly = sinf(light_elevation);
            float lz = cosf(light_elevation) * cosf(light_azimuth);
            glUniform3f(lightDirLoc, lx, ly, lz);
        }
        int ambientLoc = glGetUniformLocation(shaderProgram, "ambientStrength");
        if (ambientLoc != -1) glUniform1f(ambientLoc, ambient_strength);

        glBindVertexArray(VAO);
        if (wireframe) glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        else glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        if (g_idx_count > 0) glDrawElements(GL_TRIANGLES, g_idx_count, GL_UNSIGNED_INT, 0);
        glPointSize(8.0f);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        if (g_vert_count > 0) glDrawArrays(GL_POINTS, 0, g_vert_count);

        render_gizmo(view, proj);

        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        glDisable(GL_DEPTH_TEST);
        glViewport(0, 0, (int)main_vp->Size.x, (int)main_vp->Size.y);

        ImGui::Image((void*)(intptr_t)fbo_texture, vp_size, ImVec2(0,1), ImVec2(1,0));

        // ---- Input handling ----
        static bool orbiting = false;
        static float last_mx = 0, last_my = 0;
        static int drag_vertex = -1;
        static bool is_dragging = false;
        static int click_vertex = -1;
        static float drag_start_x=0, drag_start_y=0, drag_start_z=0;
        static ImVec2 click_mouse_pos;
        const float SENSITIVITY = 0.005f;
        const float DRAG_THRESHOLD = 3.0f;

        static bool gizmo_dragging = false;
        static int gizmo_drag_axis = -1;
        static float gizmo_prev_ndc_x = 0.0f, gizmo_prev_ndc_y = 0.0f;
        static float gizmo_screen_dir[2] = {0,0};
        static float gizmo_world_units_per_ndc = 1.0f;
        static bool undo_pushed_for_gizmo = false;
        static float gizmo_axis_vec[3] = {0,0,0};

        bool image_hovered = ImGui::IsItemHovered();
        if (image_hovered) {
            ImVec2 mouse_pos = ImGui::GetMousePos();
            ImVec2 pos = ImGui::GetItemRectMin();
            ImVec2 max = ImGui::GetItemRectMax();
            float local_x = (mouse_pos.x - pos.x) / (max.x - pos.x);
            float local_y = (mouse_pos.y - pos.y) / (max.y - pos.y);
            float ndc_x = local_x * 2.0f - 1.0f;
            float ndc_y = -(local_y * 2.0f - 1.0f);

            float fov = 3.14159f / 2.0f;
            float near = 0.1f;
            float tanHalf = tanf(fov / 2.0f);
            float A = 1.0f / (aspect * tanHalf);
            float B = 1.0f / tanHalf;
            float x_view = ndc_x * near / A;
            float y_view = ndc_y * near / B;

            float fx = -eye_x, fy = -eye_y, fz = -eye_z;
            float flen = sqrtf(fx*fx + fy*fy + fz*fz);
            bool valid_ray = (flen > 0.0001f);
            float dir_x = 0, dir_y = 0, dir_z = 0;
            float origin[3] = {eye_x, eye_y, eye_z};
            float direction[3] = {0,0,0};
            if (valid_ray) {
                fx /= flen; fy /= flen; fz /= flen;
                float ux = 0, uy = 1, uz = 0;
                float rx = uy*fz - uz*fy;
                float ry = uz*fx - ux*fz;
                float rz = ux*fy - uy*fx;
                float rlen = sqrtf(rx*rx + ry*ry + rz*rz);
                if (rlen < 0.0001f) { rx = 1; ry = 0; rz = 0; }
                else { rx /= rlen; ry /= rlen; rz /= rlen; }
                float ux2 = fy*rz - fz*ry;
                float uy2 = fz*rx - fx*rz;
                float uz2 = fx*ry - fy*rx;

                float wpx = eye_x + x_view*rx + y_view*ux2 + near*fx;
                float wpy = eye_y + x_view*ry + y_view*uy2 + near*fy;
                float wpz = eye_z + x_view*rz + y_view*uz2 + near*fz;

                dir_x = wpx - eye_x;
                dir_y = wpy - eye_y;
                dir_z = wpz - eye_z;
                float dir_len = sqrtf(dir_x*dir_x + dir_y*dir_y + dir_z*dir_z);
                if (dir_len > 0.0001f) { dir_x /= dir_len; dir_y /= dir_len; dir_z /= dir_len; }
                direction[0] = dir_x; direction[1] = dir_y; direction[2] = dir_z;
            }

            float ground_x, ground_y;
            bool ground_hit = false;
            if (valid_ray) {
                ground_hit = get_ground_intersection(ndc_x, ndc_y, ground_x, ground_y,
                                                     eye_x, eye_y, eye_z, aspect);
                if (ground_hit) {
                    g_last_ground_x = ground_x;
                    g_last_ground_y = ground_y;
                }
            }

            // Orbit
            if (ImGui::IsMouseClicked(2)) { orbiting = true; last_mx = mouse_pos.x; last_my = mouse_pos.y; }
            if (ImGui::IsMouseReleased(2)) { orbiting = false; }
            if (orbiting && ImGui::IsMouseDown(2)) {
                float dx = mouse_pos.x - last_mx;
                float dy = mouse_pos.y - last_my;
                cam_theta -= dx * SENSITIVITY;
                cam_phi -= dy * SENSITIVITY;
                if (cam_phi > 1.5f) cam_phi = 1.5f;
                if (cam_phi < -1.5f) cam_phi = -1.5f;
                last_mx = mouse_pos.x; last_my = mouse_pos.y;
            }

            // ---- Left-click ----
            if (!orbiting && valid_ray) {
                if (ImGui::IsMouseClicked(0)) {
                    click_vertex = -1;
                    is_dragging = false;
                    drag_vertex = -1;
                    gizmo_dragging = false;
                    undo_pushed_for_gizmo = false;

                    // ---- Check gizmo pick (still allowed while knife is active off-mesh) ----
                    if (g_gizmo_enabled && !g_gizmo_verts.empty() && !g_knife_active) {
                        int axis = pick_gizmo(origin, direction);
                        if (axis != -1) {
                            gizmo_dragging = true;
                            gizmo_drag_axis = axis;
                            g_gizmo_axis = axis;
                            gizmo_axis_vec[0] = 0; gizmo_axis_vec[1] = 0; gizmo_axis_vec[2] = 0;
                            gizmo_axis_vec[axis] = 1.0f;

                            gizmo_prev_ndc_x = ndc_x;
                            gizmo_prev_ndc_y = ndc_y;

                            float center_world[3] = {g_gizmo_pos[0], g_gizmo_pos[1], g_gizmo_pos[2]};
                            float axis_tip_world[3] = {center_world[0] + gizmo_axis_vec[0],
                                                       center_world[1] + gizmo_axis_vec[1],
                                                       center_world[2] + gizmo_axis_vec[2]};
                            float center_ndc[3], tip_ndc[3];
                            project_point_to_ndc(center_world, center_ndc, view, proj, viewport_width, viewport_height);
                            project_point_to_ndc(axis_tip_world, tip_ndc, view, proj, viewport_width, viewport_height);

                            float dx_ndc = tip_ndc[0] - center_ndc[0];
                            float dy_ndc = tip_ndc[1] - center_ndc[1];
                            float len = sqrtf(dx_ndc*dx_ndc + dy_ndc*dy_ndc);
                            if (len > 0.0001f) {
                                gizmo_screen_dir[0] = dx_ndc / len;
                                gizmo_screen_dir[1] = dy_ndc / len;
                                gizmo_world_units_per_ndc = 1.0f / len;
                            } else {
                                gizmo_screen_dir[0] = 1; gizmo_screen_dir[1] = 0;
                                gizmo_world_units_per_ndc = 1.0f;
                            }

                            printf("Gizmo drag start on axis %d\n", axis);
                            goto end_click_handler;
                        }
                    }

                    // ---- FIXED: Knife tool now takes exclusive priority over normal
                    // selection / vertex-add while active, so placing knife points
                    // no longer also raycasts/creates vertices via the selection path. ----
                    if (g_knife_active) {
                        float cx = 0.0f, cy = 0.0f, cz = 0.0f;
                        if (!g_vertices.empty()) {
                            size_t count = g_vertices.size() / 3;
                            for (size_t i = 0; i < count; ++i) {
                                cx += g_vertices[i*3];
                                cy += g_vertices[i*3+1];
                                cz += g_vertices[i*3+2];
                            }
                            cx /= (float)count;
                            cy /= (float)count;
                            cz /= (float)count;
                        }
                        float plane_center[3] = {cx, cy, cz};

                        float t_plane = -((origin[0]-plane_center[0])*direction[0] +
                                          (origin[1]-plane_center[1])*direction[1] +
                                          (origin[2]-plane_center[2])*direction[2]);

                        if (t_plane > 0) {
                            float hit[3] = {
                                origin[0] + t_plane * direction[0],
                                origin[1] + t_plane * direction[1],
                                origin[2] + t_plane * direction[2]
                            };

                            if (g_knife_stage == 0) {
                                g_knife_start[0] = hit[0];
                                g_knife_start[1] = hit[1];
                                g_knife_start[2] = hit[2];
                                g_knife_stage = 1;
                                printf("Knife start: (%f, %f, %f)\n", hit[0], hit[1], hit[2]);
                            } else if (g_knife_stage == 1) {
                                g_knife_end[0] = hit[0];
                                g_knife_end[1] = hit[1];
                                g_knife_end[2] = hit[2];
                                g_knife_stage = 2;
                                printf("Knife end: (%f, %f, %f)\n", hit[0], hit[1], hit[2]);

                                push_undo();
                                knife_cut(g_knife_start, g_knife_end);

                                g_knife_active = false;
                                g_knife_stage = 0;
                                printf("Knife cut executed\n");
                            }
                        }
                        // Consume the click entirely — do not fall through to
                        // normal vertex/edge/face selection below.
                        goto end_click_handler;
                    }

                    // ---- Normal selection (only reached when knife is NOT active) ----
                    if (g_sel_mode == SELECT_VERTEX) {
                        click_vertex = raycast_vertex(eye_x, eye_y, eye_z, dir_x, dir_y, dir_z, 0.2f*0.2f);
                    } else if (g_sel_mode == SELECT_EDGE) {
                        auto edge = pick_edge(origin, direction, 0.15f);
                        if (edge.first != -1) {
                            if (!ImGui::GetIO().KeyShift) g_selected_edges.clear();
                            g_selected_edges.push_back(edge);
                            g_selected.clear();
                            g_selected.push_back(edge.first);
                            g_selected.push_back(edge.second);
                            click_vertex = -3;
                            update_mesh_buffers();
                            update_gizmo_selection();
                            printf("Selected edge (%d, %d)\n", edge.first, edge.second);
                        } else {
                            if (!ImGui::GetIO().KeyShift) g_selected_edges.clear();
                            update_mesh_buffers();
                            update_gizmo_selection();
                        }
                    } else if (g_sel_mode == SELECT_FACE) {
                        float u, v;
                        int tri = pick_face(origin, direction, u, v);
                        if (tri != -1) {
                            int face_id = -1;
                            for (size_t i = 0; i < g_face_list.size(); i++) {
                                for (int t : g_face_list[i].tri_indices) {
                                    if (t == tri) {
                                        face_id = (int)i;
                                        break;
                                    }
                                }
                                if (face_id != -1) break;
                            }
                            if (face_id != -1) {
                                if (!ImGui::GetIO().KeyShift) g_selected_faces.clear();
                                for (int t : g_face_list[face_id].tri_indices) {
                                    g_selected_faces.push_back(t);
                                }
                                g_selected.clear();
                                for (int t : g_selected_faces) {
                                    g_selected.push_back((int)g_indices[t*3]);
                                    g_selected.push_back((int)g_indices[t*3+1]);
                                    g_selected.push_back((int)g_indices[t*3+2]);
                                }
                                std::sort(g_selected.begin(), g_selected.end());
                                g_selected.erase(std::unique(g_selected.begin(), g_selected.end()), g_selected.end());

                                update_mesh_buffers();
                                update_gizmo_selection();
                                printf("Selected face %d (%zu triangles)\n", face_id, g_face_list[face_id].tri_indices.size());
                            }
                        } else {
                            if (!ImGui::GetIO().KeyShift) {
                                g_selected_faces.clear();
                                update_mesh_buffers();
                                update_gizmo_selection();
                            }
                        }
                    }
                    click_mouse_pos = mouse_pos;
                    end_click_handler:;
                }

                // ---- Dragging (gizmo / vertex) ----
                if (gizmo_dragging && ImGui::IsMouseDown(0)) {
                    float delta_ndc_x = ndc_x - gizmo_prev_ndc_x;
                    float delta_ndc_y = ndc_y - gizmo_prev_ndc_y;

                    float projected_ndc = delta_ndc_x * gizmo_screen_dir[0] + delta_ndc_y * gizmo_screen_dir[1];
                    float world_delta = projected_ndc * gizmo_world_units_per_ndc;

                    if (fabs(world_delta) > 0.0001f && !g_gizmo_verts.empty()) {
                        if (!undo_pushed_for_gizmo) {
                            push_undo();
                            undo_pushed_for_gizmo = true;
                        }
                        for (int v : g_gizmo_verts) {
                            g_vertices[v*3] += gizmo_axis_vec[0] * world_delta;
                            g_vertices[v*3+1] += gizmo_axis_vec[1] * world_delta;
                            g_vertices[v*3+2] += gizmo_axis_vec[2] * world_delta;
                        }
                        g_gizmo_pos[0] += gizmo_axis_vec[0] * world_delta;
                        g_gizmo_pos[1] += gizmo_axis_vec[1] * world_delta;
                        g_gizmo_pos[2] += gizmo_axis_vec[2] * world_delta;
                        update_mesh_buffers();
                    }
                    gizmo_prev_ndc_x = ndc_x;
                    gizmo_prev_ndc_y = ndc_y;
                }

                // Vertex drag (skip entirely while knife is active)
                if (!gizmo_dragging && !g_knife_active && g_sel_mode == SELECT_VERTEX) {
                    if (ImGui::IsMouseDown(0) && !ImGui::IsMouseClicked(0)) {
                        if (click_vertex != -1) {
                            float dx = mouse_pos.x - click_mouse_pos.x;
                            float dy = mouse_pos.y - click_mouse_pos.y;
                            float dist = sqrtf(dx*dx + dy*dy);
                            if (dist > DRAG_THRESHOLD && !is_dragging) {
                                push_undo();
                                is_dragging = true;
                                drag_vertex = click_vertex;
                                drag_start_x = g_vertices[drag_vertex*3];
                                drag_start_y = g_vertices[drag_vertex*3+1];
                                drag_start_z = g_vertices[drag_vertex*3+2];
                            }
                        }
                    }

                    if (ImGui::IsMouseDown(0) && is_dragging && drag_vertex != -1) {
                        float p0x = drag_start_x, p0y = drag_start_y, p0z = drag_start_z;
                        float nx = fx, ny = fy, nz = fz;
                        float denom = dir_x*nx + dir_y*ny + dir_z*nz;
                        if (fabs(denom) > 0.0001f) {
                            float t = ((p0x - eye_x)*nx + (p0y - eye_y)*ny + (p0z - eye_z)*nz) / denom;
                            if (t >= 0) {
                                float new_x = eye_x + t * dir_x;
                                float new_y = eye_y + t * dir_y;
                                float new_z = eye_z + t * dir_z;
                                g_vertices[drag_vertex*3]     = new_x;
                                g_vertices[drag_vertex*3+1]   = new_y;
                                g_vertices[drag_vertex*3+2]   = new_z;
                                update_mesh_buffers();
                                update_gizmo_selection();
                            }
                        }
                    }
                }

                // ---- Mouse Release ----
                if (ImGui::IsMouseReleased(0)) {
                    if (gizmo_dragging) {
                        gizmo_dragging = false;
                        g_gizmo_axis = -1;
                        undo_pushed_for_gizmo = false;
                        printf("Gizmo drag finished\n");
                        goto end_mouse_release;
                    }

                    // FIXED: guarded so releasing the mouse while placing a knife
                    // point can no longer fall through and spawn a stray vertex
                    // via the "no vertex hit -> add_vertex on ground" path.
                    if (!g_knife_active && g_sel_mode == SELECT_VERTEX) {
                        if (!is_dragging) {
                            if (click_vertex != -1) {
                                if (!ImGui::GetIO().KeyShift) g_selected.clear();
                                g_selected.push_back(click_vertex);
                                g_selected.erase(std::unique(g_selected.begin(), g_selected.end()), g_selected.end());
                                update_mesh_buffers();
                                update_gizmo_selection();
                            } else {
                                if (ground_hit) {
                                    push_undo();
                                    if (!ImGui::GetIO().KeyShift) g_selected.clear();
                                    add_vertex(g_last_ground_x, g_last_ground_y);
                                    g_selected.push_back(g_vert_count - 1);
                                    update_mesh_buffers();
                                    update_gizmo_selection();
                                } else {
                                    if (!ImGui::GetIO().KeyShift) {
                                        g_selected.clear();
                                        update_mesh_buffers();
                                        update_gizmo_selection();
                                    }
                                }
                            }
                        }
                    }
                    end_mouse_release:;
                    click_vertex = -1;
                    is_dragging = false;
                    drag_vertex = -1;

                    // ---- Draw Face: collect points ----
                    if (g_draw_face_active && !gizmo_dragging && valid_ray && ImGui::IsMouseClicked(0)) {
                        // Project the click onto the selected face's plane
                        if (g_draw_face_face_idx >= 0 && g_draw_face_face_idx < (int)g_face_list.size()) {
                            float nx, ny, nz, d;
                            get_face_plane(g_draw_face_face_idx, nx, ny, nz, d);
                            float denom = nx*direction[0] + ny*direction[1] + nz*direction[2];
                            if (fabs(denom) > 1e-6f) {
                                float t = -(nx*origin[0] + ny*origin[1] + nz*origin[2] + d) / denom;
                                if (t > 0) {
                                    std::array<float,3> hit = {origin[0] + t*direction[0],
                                                               origin[1] + t*direction[1],
                                                               origin[2] + t*direction[2]};
                                    g_draw_face_points.push_back(hit);
                                    printf("Point %zu: (%.3f, %.3f, %.3f)\n", g_draw_face_points.size(),
                                           hit[0], hit[1], hit[2]);
                                }
                            }
                        }
                    }
                }
            }
        }
    }

end_viewport:
    ImGui::End();
}