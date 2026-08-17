// properties.cpp
#include "properties.h"
#include "globals.h"
#include "mesh.h"
#include "undo.h"   // <-- NEW
#include <algorithm>
#include "imgui/imgui.h"

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

    ImGui::Separator();
    ImGui::Text("Lighting");
    ImGui::SliderAngle("Azimuth", &light_azimuth, -180.0f, 180.0f);
    ImGui::SliderAngle("Elevation", &light_elevation, -90.0f, 90.0f);
    ImGui::SliderFloat("Ambient", &ambient_strength, 0.0f, 0.5f);

    ImGui::Separator();
    ImGui::Text("Normals");
    if (ImGui::Button("Recalc Normals")) {
        push_undo(); // Normal recalculation changes the mesh appearance
        compute_normals();
        update_mesh_buffers();
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

    ImGui::End();
}