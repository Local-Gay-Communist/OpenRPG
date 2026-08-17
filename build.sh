#!/bin/bash
echo "Compiling OpenRPG Editor..."
CPPFLAGS="-include cmath -include algorithm"
g++ -std=c++17 -I. -Iimgui -Iimgui/backends \
    $CPPFLAGS \
    grid.cpp \
    main.cpp \
    math.cpp \
    shader.cpp \
    fbo.cpp \
    mesh.cpp \
    viewport.cpp \
    hierarchy.cpp \
    properties.cpp \
    output.cpp \
    imgui/imgui.cpp \
    imgui/imgui_draw.cpp \
    imgui/imgui_widgets.cpp \
    imgui/imgui_tables.cpp \
    imgui/backends/imgui_impl_glfw.cpp \
    imgui/backends/imgui_impl_opengl3.cpp \
    -lglfw -lGLEW -lGL -ldl -lpthread \
    -o OpenRPG_Editor

if [ $? -eq 0 ]; then
    echo "Success! Run ./OpenRPG_Editor"
else
    echo "Build failed."
fi