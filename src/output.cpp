// output.cpp
#include "output.h"
#include "globals.h"
#include "imgui/imgui.h"

void OutputWindow() {
    ImGuiViewport* main_vp = ImGui::GetMainViewport();
    ImVec2 work_pos = main_vp->WorkPos;
    ImVec2 work_size = main_vp->WorkSize;

    ImGui::SetNextWindowPos(ImVec2(work_pos.x + work_size.x * 0.20f, work_pos.y + work_size.y * 0.80f), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(work_size.x * 0.55f, work_size.y * 0.20f), ImGuiCond_FirstUseEver);
    ImGui::Begin("Output");
    ImGui::Text("Triangles: %d", g_idx_count / 3);
    ImGui::Text("Selected: %zu", g_selected.size());
    ImGui::End();
}