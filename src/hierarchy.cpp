// hierarchy.cpp
#include "hierarchy.h"
#include "globals.h"
#include "mesh.h"
#include <string>
#include <algorithm> 
#include "imgui/imgui.h"

void HierarchyWindow() {
    ImGuiViewport* main_vp = ImGui::GetMainViewport();
    ImVec2 work_pos = main_vp->WorkPos;
    ImVec2 work_size = main_vp->WorkSize;

    ImGui::SetNextWindowPos(ImVec2(work_pos.x + 0, work_pos.y + 0), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(work_size.x * 0.20f, work_size.y * 0.80f), ImGuiCond_FirstUseEver);
    ImGui::Begin("Entity Hierarchy");

    // Display mode
    const char* mode_names[] = {"", "VERTEX", "EDGE", "FACE"};
    ImGui::Text("Mode: %s", mode_names[g_sel_mode]);
    ImGui::Separator();

    ImGui::Text("Vertices (%d)", g_vert_count);
    ImGui::Text("Hotkeys: F=Fill, Delete=Remove, Esc=Deselect");
    for(size_t i = 0; i < g_vertices.size() / 3; i++) {
        bool sel = std::find(g_selected.begin(), g_selected.end(), (int)i) != g_selected.end();
        if(ImGui::Selectable(("V" + std::to_string(i)).c_str(), sel)) {
            if(!ImGui::GetIO().KeyShift) g_selected.clear();
            g_selected.push_back((int)i);
            g_selected.erase(std::unique(g_selected.begin(), g_selected.end()), g_selected.end());
            update_mesh_buffers();
        }
    }
    ImGui::End();
}