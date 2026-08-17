cmake_minimum_required(VERSION 3.10)
project(OpenRPG_Editor VERSION 1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Force-include cmath and algorithm for all sources
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -include cmath -include algorithm")

if(MINGW)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpermissive")
    set(GLFW_LIBRARIES glfw3)
    set(GLEW_LIBRARIES glew32)
    set(OPENGL_LIBRARIES opengl32)
    link_directories(/usr/x86_64-w64-mingw32/lib)
else()
    find_package(OpenGL REQUIRED)
    find_package(glfw3 REQUIRED)
    find_package(GLEW REQUIRED)
endif()

# --- Set imgui location ---
set(IMGUI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/imgui)

# --- Source files ---
set(SRC_FILES
    src/main.cpp
    src/math.cpp
    src/shader.cpp
    src/fbo.cpp
    src/mesh.cpp
    src/grid.cpp
    src/viewport.cpp
    src/hierarchy.cpp
    src/properties.cpp
    src/output.cpp
    src/undo.cpp          # <-- ADD THIS LINE
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
    ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
)

# --- Executable ---
add_executable(OpenRPG_Editor ${SRC_FILES})

# --- Include directories ---
target_include_directories(OpenRPG_Editor PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/external
    ${IMGUI_DIR}
    ${IMGUI_DIR}/backends
)

# --- Compile definitions ---
if(MINGW)
    target_compile_definitions(OpenRPG_Editor PRIVATE GLEW_STATIC)
endif()

# --- Link libraries ---
if(MINGW)
    target_link_libraries(OpenRPG_Editor
        ${GLFW_LIBRARIES}
        ${GLEW_LIBRARIES}
        ${OPENGL_LIBRARIES}
        ${CMAKE_DL_LIBS}
        pthread
        ws2_32
        winmm
        gdi32
    )
else()
    target_link_libraries(OpenRPG_Editor
        ${OPENGL_LIBRARIES}
        glfw
        GLEW::GLEW
        ${CMAKE_DL_LIBS}
        pthread
    )
endif()

# --- Output directory ---
set_target_properties(OpenRPG_Editor PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

install(TARGETS OpenRPG_Editor DESTINATION bin)