// properties.cpp
#include "properties.h"
#include "globals.h"
#include "mesh.h"
#include "undo.h"   // <-- NEW
#include <algorithm>
#include "imgui/imgui.h"
#include "viewport.h"

void PropertiesWindow() {
    ImGuiViewport* main_vp = ImGui::GetMainViewport();
    ImVec2 work_pos = main_vp->WorkPos;
    ImVec2 work_size = main_vp->WorkSize;

    ImGui::SetNextWindowPos(ImVec2(work_pos.x + work_size.x * 0.75f, work_pos.y + 0), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(work_size.x * 0.25f, work_size.y * 0.80f), ImGuiCond_FirstUseEver);
    ImGui::Begin("Properties");

    if (g_sel_mode == SELECT_EDGE) {
    ImGui::Text("Selected Edges: %zu", g_selected_edges.size());
    for (size_t i=0; i<g_selected_edges.size(); i++) {
        auto& e = g_selected_edges[i];
        ImGui::Text("  Edge %zu: (%d, %d)", i, e.first, e.second);
    }
    }
    if (g_sel_mode == SELECT_FACE) {
        ImGui::Text("Selected Faces: %zu", g_selected_faces.size());
        for (size_t i=0; i<g_selected_faces.size(); i++) {
            int tri = g_selected_faces[i];
            ImGui::Text("  Face %zu: tri %d (v%d, v%d, v%d)", 
                        i, tri, g_indices[tri*3], g_indices[tri*3+1], g_indices[tri*3+2]);
        }
    }

    if(g_selected.size() == 1) {
        int idx = g_selected[0];
        ImGui::Text("Vertex %d", idx);
        float x = g_vertices[idx*3], y = g_vertices[idx*3+1], z = g_vertices[idx*3+2];
        if(ImGui::DragFloat("X", &x, 0.01f)) { 
            push_undo(); // Changing a vertex coordinate is undoable
            g_vertices[idx*3] = x; 
            update_mesh_buffers(); 
        }
        if(ImGui::DragFloat("Y", &y, 0.01f)) { 
            push_undo();
            g_vertices[idx*3+1] = y; 
            update_mesh_buffers(); 
        }
        if(ImGui::DragFloat("Z", &z, 0.01f)) { 
            push_undo();
            g_vertices[idx*3+2] = z; 
            update_mesh_buffers(); 
        }
        ImGui::Text("Normal: (%.2f, %.2f, %.2f)", g_normals[idx*3], g_normals[idx*3+1], g_normals[idx*3+2]);
    } else if(g_selected.size() == 3) {
        ImGui::Text("3 vertices selected. Press F to fill.");
    } else if(g_selected.size() == 2) {
        ImGui::Text("2 vertices selected. Press F to fill (creates new vertex).");
    } else {
        ImGui::Text("Select 2 or 3 vertices, then press F.");
    }

    ImGui::Separator();
    ImGui::SliderFloat("Snap Dist", &SNAP_DIST, 0.01f, 0.2f, "%.2f");

    if(ImGui::Button("Delete Selected (Del)")) {
        if (!g_selected.empty()) {
            push_undo();
            std::sort(g_selected.begin(), g_selected.end(), std::greater<int>());
            for(int idx : g_selected) remove_vertex(idx);
            g_selected.clear();
            update_mesh_buffers();
        }
    }
    ImGui::SameLine();
    if(ImGui::Button("Duplicate")) {
        if (!g_selected.empty()) {
            push_undo();
            duplicate_selected();
        }
    }
    if(ImGui::Button("Clear All (Wipe)")) {
        if (!g_vertices.empty()) {
            push_undo();
            g_vertices.clear(); g_normals.clear(); g_indices.clear(); g_selected.clear();
            update_mesh_buffers();
        }
    }

    ImGui::Separator();
    ImGui::Text("Polygon Tools:");
    if (ImGui::Button("Square (2 verts)")) {
        if (g_selected.size() >= 2) {
            push_undo();
            create_square_from_selected();
        }
    }
    ImGui::SameLine();
    if (ImGui::Button("Cube (1 vert)")) {
        if (!g_selected.empty()) {
            push_undo();
            create_cube_from_selected();
        }
    }
    ImGui::SameLine();
    if (ImGui::Button("Bridge")) {
        if (g_selected.size() >= 6) {
            push_undo();
            bridge_polygons();
        }
    }
    ImGui::SameLine();
    if (ImGui::Button("Detriangulate")) {
        if (!g_selected.empty()) {
            push_undo();
            detriangulate_selected();
        }
    }
    ImGui::Text("Face Tools");
    if (ImGui::Button("Make Face from Selected")) {
        if (g_selected.size() >= 3) {
            push_undo();
            make_face_from_selected();
            update_gizmo_selection();
        } else {
            printf("Select at least 3 vertices.\n");
        }
    }

    ImGui::Separator();
    ImGui::Text("Lighting");
    ImGui::SliderAngle("Azimuth", &light_azimuth, -180.0f, 180.0f);
    ImGui::SliderAngle("Elevation", &light_elevation, -90.0f, 90.0f);
    ImGui::SliderFloat("Ambient", &ambient_strength, 0.0f, 0.5f);

    ImGui::Separator();
    ImGui::Text("Normals");
    if (ImGui::Button("Recalc Normals")) {
        push_undo();
        compute_normals();
        update_mesh_buffers();
        printf("Normals recalculated\n");
    }
    ImGui::SameLine();
    if (ImGui::Button("Flip Selected")) {
        if (!g_selected.empty()) {
            push_undo();
            for (int idx : g_selected) {
                g_normals[idx*3] = -g_normals[idx*3];
                g_normals[idx*3+1] = -g_normals[idx*3+1];
                g_normals[idx*3+2] = -g_normals[idx*3+2];
            }
            update_mesh_buffers();
        }
    }
    if (ImGui::Button("Unify Normals")) {
        if (!g_indices.empty()) {
            push_undo();
            unify_normals();
        }
    }

    ImGui::Separator();
    ImGui::Text("Extrude");
    if (ImGui::Button("Extrude Faces (E)")) {
        if (!g_selected_faces.empty()) {
            push_undo();
            extrude_selected();
        }
    }

    if (ImGui::Button("Weld Vertices")) {
        push_undo();
        weld_vertices(0.001f);
    }

    ImGui::Text("Faces: %zu", g_face_list.size());
    ImGui::Text("Triangles: %d", g_idx_count / 3);

    // ---- Edge splitting ----
    if (g_selected_edges.size() == 1) {
        ImGui::Separator();
        ImGui::Text("Selected Edge");
        int v0 = g_selected_edges[0].first;
        int v1 = g_selected_edges[0].second;
        ImGui::Text("Vertices: (%d, %d)", v0, v1);

        static float split_t = 0.5f;
        ImGui::SliderFloat("Split Position", &split_t, 0.0f, 1.0f, "%.2f");

        if (ImGui::Button("Split Edge")) {
            if (v0 >= 0 && v1 >= 0 && v0 < g_vert_count && v1 < g_vert_count) {
                push_undo();
                split_edge(v0, v1, split_t);
                // Clear the edge selection after split
                g_selected_edges.clear();
                g_selected.clear();
                update_mesh_buffers();
                update_gizmo_selection();
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("Split at Midpoint")) {
            split_t = 0.5f;
            if (v0 >= 0 && v1 >= 0 && v0 < g_vert_count && v1 < g_vert_count) {
                push_undo();
                split_edge(v0, v1, 0.5f);
                g_selected_edges.clear();
                g_selected.clear();
                update_mesh_buffers();
                update_gizmo_selection();
            }
        }
    }

    ImGui::Separator();
    ImGui::Text("Weld");

    if (ImGui::Button("Weld by Distance")) {
        if (!g_vertices.empty()) {
            push_undo();
            weld_vertices(0.001f, false, false);
        }
    }
    ImGui::SameLine();
    if (ImGui::Button("Weld to Center")) {
        if (g_selected.size() >= 2) {
            push_undo();
            weld_vertices(0.0f, true, false);
        }
    }
    ImGui::SameLine();
    if (ImGui::Button("Weld to First")) {
        if (g_selected.size() >= 2) {
            push_undo();
            weld_vertices(0.0f, false, true);
        }
    }

    ImGui::Separator();
    ImGui::Text("Knife Tool");
    if (ImGui::Button(g_knife_active ? "Cancel Knife" : "Activate Knife")) {
        g_knife_active = !g_knife_active;
        if (g_knife_active) {
            g_knife_stage = 0;
            printf("Knife mode activated – click two points in viewport\n");
        } else {
            g_knife_stage = 0;
        }
    }
    if (g_knife_active) {
        ImGui::TextColored(ImVec4(1,1,0,1), "Click two points to define cut line.");
        ImGui::Text("Stage: %s", g_knife_stage == 0 ? "first point" : "second point");
    }

    // ---- Face-specific tools ----
    if (!g_selected_faces.empty()) {
        ImGui::Separator();
        ImGui::Text("Selected Faces: %zu", g_selected_faces.size());

        if (g_selected_faces.size() == 1) {
            int face_group_idx = -1;
            for (size_t i = 0; i < g_face_list.size(); ++i) {
                for (int tri : g_face_list[i].tri_indices) {
                    if (tri == g_selected_faces[0]) {
                        face_group_idx = (int)i;
                        break;
                    }
                }
                if (face_group_idx != -1) break;
            }

            if (face_group_idx != -1) {
                // ---- Draw Face ----
                if (ImGui::Button(g_draw_face_active ? "Cancel Draw Face" : "Draw Face")) {
                    if (!g_draw_face_active) {
                        g_draw_face_active = true;
                        g_draw_face_face_idx = face_group_idx;
                        g_draw_face_points.clear();
                        printf("Draw Face mode activated – click points on the face, then press Finish.\n");
                    } else {
                        g_draw_face_active = false;
                        g_draw_face_face_idx = -1;
                        g_draw_face_points.clear();
                    }
                }
                ImGui::SameLine();
                if (g_draw_face_active) {
                    ImGui::TextColored(ImVec4(1,1,0,1), "%zu points", g_draw_face_points.size());
                    if (g_draw_face_points.size() >= 3) {
                        if (ImGui::Button("Finish Draw Face")) {
                            push_undo();
                            split_face_by_polygon(face_group_idx, g_draw_face_points);
                            g_draw_face_active = false;
                            g_draw_face_face_idx = -1;
                            g_draw_face_points.clear();
                        }
                        ImGui::SameLine();
                        if (ImGui::Button("Clear Points")) {
                            g_draw_face_points.clear();
                        }
                    } else {
                        ImGui::Text("Need at least 3 points.");
                    }
                }

                // ---- Also keep the old "Cut Face" line-split tool ----
                // (optional)
            }
        }
    }

    ImGui::End();   
}