// main.cpp
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include "imgui/imgui.h"
#include "imgui/backends/imgui_impl_glfw.h"
#include "imgui/backends/imgui_impl_opengl3.h"

#include <algorithm>
#include <string>

#include "globals.h"
#include "math.h"
#include "shader.h"
#include "fbo.h"
#include "mesh.h"
#include "viewport.h"
#include "hierarchy.h"
#include "properties.h"
#include "output.h"
#include "grid.h"
#include "undo.h"

// ---- Define globals ----
std::vector<float> g_vertices;
std::vector<float> g_normals;
std::vector<unsigned int> g_indices;
std::vector<int> g_selected;
int g_vert_count = 0, g_idx_count = 0;
float SNAP_DIST = 0.05f;
float g_last_ground_x = 0.0f, g_last_ground_y = 0.0f;

unsigned int VAO, VBO, EBO;
unsigned int fbo_texture, fbo_renderbuffer, fbo;
int viewport_width = 1280, viewport_height = 720;
unsigned int shaderProgram;

float cam_theta = 0.0f, cam_phi = 0.0f, cam_dist = 4.0f;
bool wireframe = false;

float light_azimuth = 0.5f;
float light_elevation = 0.8f;
float ambient_strength = 0.25f;

SelectionMode g_sel_mode = SELECT_VERTEX;
std::vector<std::pair<int, int>> g_selected_edges;
std::vector<int> g_selected_faces;

bool g_gizmo_enabled = true;
int g_gizmo_axis = -1;
float g_gizmo_pos[3] = {0.0f, 0.0f, 0.0f};
std::vector<int> g_gizmo_verts;
float g_gizmo_size = 0.8f;

float g_bevel_width = 0.1f;

std::vector<MeshFace> g_face_list;

bool g_knife_active = false;
int g_knife_stage = 0;
float g_knife_start[3] = {0.0f, 0.0f, 0.0f};
float g_knife_end[3] = {0.0f, 0.0f, 0.0f};

bool g_draw_face_active = false;
int g_draw_face_face_idx = -1;
std::vector<std::array<float,3>> g_draw_face_points;


int main() {
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_COMPAT_PROFILE);
    GLFWwindow* window = glfwCreateWindow(1600, 900, "OpenRPG Workshop", NULL, NULL);
    if(!window) return -1;
    glfwMakeContextCurrent(window);

    glewExperimental = GL_TRUE;
    if(glewInit() != GLEW_OK) return -1;
    glViewport(0, 0, 1600, 900);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");

    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);
    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(float), (void*)(6 * sizeof(float)));
    glEnableVertexAttribArray(2);
    glBindVertexArray(0);

    shaderProgram = create_program();

    add_vertex(-0.5f, -0.5f);
    add_vertex( 0.5f, -0.5f);
    add_vertex( 0.5f,  0.5f);
    add_vertex(-0.5f,  0.5f);
    add_triangle(0, 1, 2);
    add_triangle(0, 2, 3);
    update_mesh_buffers();
    update_gizmo_selection();

    init_grid();
    init_fbo(1600, 900);

    while(!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        if(ImGui::BeginMainMenuBar()) {
            if(ImGui::BeginMenu("File")) {
                if(ImGui::MenuItem("New Project")) {
                    push_undo();
                    g_vertices.clear(); g_normals.clear(); g_indices.clear(); g_selected.clear(); g_face_list.clear();  
                    g_selected_edges.clear();
                    g_selected_faces.clear();
                    g_gizmo_verts.clear();
                    g_gizmo_pos[0] = g_gizmo_pos[1] = g_gizmo_pos[2] = 0.0f;
                    g_gizmo_axis = -1;
                    update_mesh_buffers();
                    update_gizmo_selection();
                    clear_undo_stack();
                    printf("New project created\n");
                }
                if(ImGui::MenuItem("Save")) {
                    save_project_binary("project.rpg");
                }
                if(ImGui::MenuItem("Load")) {
                    load_project_binary("project.rpg");
                    clear_undo_stack();
                    update_gizmo_selection();
                }
                if(ImGui::MenuItem("Export OBJ")) {
                    export_obj("export.obj");
                }
                ImGui::Separator();
                if(ImGui::MenuItem("Exit")) {
                    glfwSetWindowShouldClose(window, GLFW_TRUE);
                }
                ImGui::EndMenu();
            }
            if(ImGui::BeginMenu("Edit")) {
                if (ImGui::MenuItem("Undo", "Ctrl+Z", false, can_undo())) {
                    undo();
                }
                if (ImGui::MenuItem("Redo", "Ctrl+Y", false, can_redo())) {
                    redo();
                }
                ImGui::Separator();
                ImGui::EndMenu();
            }
            if(ImGui::BeginMenu("View")) {
                ImGui::MenuItem("Wireframe", NULL, &wireframe);
                ImGui::MenuItem("Gizmo", NULL, &g_gizmo_enabled);
                ImGui::EndMenu();
            }
            if(ImGui::BeginMenu("Tools")) {
                if(ImGui::MenuItem("Mesh Editor")) {}
                if(ImGui::MenuItem("Animation Timeline")) {}
                ImGui::EndMenu();
            }
            ImGui::EndMainMenuBar();
        }

        RenderViewportWindow();
        HierarchyWindow();
        PropertiesWindow();
        OutputWindow();

        // ---- Undo/Redo Hotkeys ----
        if (ImGui::IsKeyPressed(ImGuiKey_Z) && ImGui::GetIO().KeyCtrl && !ImGui::GetIO().KeyShift && !ImGui::IsAnyItemActive()) {
            undo();
        }
        if (ImGui::IsKeyPressed(ImGuiKey_Y) && ImGui::GetIO().KeyCtrl && !ImGui::IsAnyItemActive()) {
            redo();
        }
        if (ImGui::IsKeyPressed(ImGuiKey_Z) && ImGui::GetIO().KeyCtrl && ImGui::GetIO().KeyShift && !ImGui::IsAnyItemActive()) {
            redo();
        }

        // ---- Selection Mode Hotkeys ----
        if (ImGui::IsKeyPressed(ImGuiKey_1) && !ImGui::IsAnyItemActive()) {
            g_sel_mode = SELECT_VERTEX;
            update_gizmo_selection();
            printf("Mode: VERTEX\n");
        }
        if (ImGui::IsKeyPressed(ImGuiKey_2) && !ImGui::IsAnyItemActive()) {
            g_sel_mode = SELECT_EDGE;
            update_gizmo_selection();
            printf("Mode: EDGE\n");
        }
        if (ImGui::IsKeyPressed(ImGuiKey_3) && !ImGui::IsAnyItemActive()) {
            g_sel_mode = SELECT_FACE;
            update_gizmo_selection();
            printf("Mode: FACE\n");
        }

        // ---- Hotkeys ----
        if (ImGui::IsKeyPressed(ImGuiKey_F) && !ImGui::IsAnyItemActive()) {
            push_undo();
            if (g_selected.size() == 2) {
                int v0 = g_selected[0], v1 = g_selected[1];
                if (v0 != v1) {
                    add_vertex(g_last_ground_x, g_last_ground_y);
                    int v2 = (int)g_vertices.size() / 3 - 1;
                    if (v0 != v2 && v1 != v2) {
                        add_triangle(v0, v1, v2);
                        g_selected.clear();
                        update_mesh_buffers();
                        update_gizmo_selection();
                    }
                }
            } else if (g_selected.size() == 3) {
                add_triangle(g_selected[0], g_selected[1], g_selected[2]);
                g_selected.clear();
                update_mesh_buffers();
                update_gizmo_selection();
            } else if (g_selected.size() >= 4) {
                make_face_from_selected();
                // (make_face_from_selected already calls update_mesh_buffers)
            } else {
                printf("Select at least 2 vertices to fill.\n");
            }
        }

        if ((ImGui::IsKeyPressed(ImGuiKey_Delete) || ImGui::IsKeyPressed(ImGuiKey_Backspace)) && !ImGui::IsAnyItemActive()) {
            if (!g_selected.empty()) {
                push_undo();
                std::sort(g_selected.begin(), g_selected.end(), std::greater<int>());
                for (int idx : g_selected) remove_vertex(idx);
                g_selected.clear();
                g_selected_edges.clear();
                g_selected_faces.clear();
                update_mesh_buffers();
                update_gizmo_selection();
            }
        }

        if (ImGui::IsKeyPressed(ImGuiKey_Escape)) {
            g_selected.clear();
            g_selected_edges.clear();
            g_selected_faces.clear();
            update_mesh_buffers();
            update_gizmo_selection();
        }

        // Polygon tool hotkeys
        if (ImGui::IsKeyPressed(ImGuiKey_S) && ImGui::GetIO().KeyShift && !ImGui::IsAnyItemActive()) {
            push_undo();
            create_square_from_selected();
            update_gizmo_selection();
        }
        if (ImGui::IsKeyPressed(ImGuiKey_C) && ImGui::GetIO().KeyShift && !ImGui::IsAnyItemActive()) {
            push_undo();
            create_cube_from_selected();
            update_gizmo_selection();
        }
        if (ImGui::IsKeyPressed(ImGuiKey_B) && ImGui::GetIO().KeyShift && !ImGui::IsAnyItemActive()) {
            push_undo();
            bridge_polygons();
            update_gizmo_selection();
        }
        if (ImGui::IsKeyPressed(ImGuiKey_D) && ImGui::GetIO().KeyShift && !ImGui::IsAnyItemActive()) {
            push_undo();
            detriangulate_selected();
            update_gizmo_selection();
        }

        if (ImGui::IsKeyPressed(ImGuiKey_E) && !ImGui::IsAnyItemActive()) {
            if (g_sel_mode == SELECT_FACE && !g_selected_faces.empty()) {
                push_undo();
                extrude_selected();
            } else if (g_sel_mode == SELECT_EDGE && !g_selected_edges.empty()) {
                // Edge extrude not implemented yet, but we can add a message
                printf("Edge extrude not implemented yet\n");
            } else if (g_sel_mode == SELECT_VERTEX && !g_selected.empty()) {
                // Vertex extrude not implemented yet
                printf("Vertex extrude not implemented yet\n");
            }
        }

        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        glClearColor(0.12f, 0.12f, 0.16f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}