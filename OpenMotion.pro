
QT += core gui widgets openglwidgets network serialport

# --- Resolve HOME robustly ---
HOME_DIR = $$(HOME)
isEmpty(HOME_DIR) {
    HOME_DIR = $$[env.HOME]
}
isEmpty(HOME_DIR) {
    error("HOME environment variable not set; use absolute paths instead")
}

#Eigen
# Eigen
INCLUDEPATH += /usr/include/eigen3
# ImGui
IMGUI_DIR = "$$HOME_DIR/Downloads/imgui-master"
INCLUDEPATH += "$$IMGUI_DIR" "$$IMGUI_DIR/backends"

# ImPlot3D
IMPLOT3D_DIR = "$$HOME_DIR/Downloads/implot3d"
INCLUDEPATH += "$$IMPLOT3D_DIR"

# --- Sanity checks ---
!exists("$$IMGUI_DIR/imgui.cpp") {
    error("Can't find $$IMGUI_DIR/imgui.cpp — fix IMGUI_DIR path")
}
!exists("$$IMGUI_DIR/backends/imgui_impl_opengl3.cpp") {
    error("Can't find $$IMGUI_DIR/backends/imgui_impl_opengl3.cpp — fix IMGUI_DIR path")
}
!exists("$$IMPLOT3D_DIR/implot3d.cpp") {
    error("Can't find $$IMPLOT3D_DIR/implot3d.cpp — fix IMPLOT3D_DIR path")
}

# Sources
SOURCES += \
    main.cpp \
    mainwindow.cpp \
    ImGuiGLWidget.cpp \
    "$$IMGUI_DIR/imgui.cpp" \
    "$$IMGUI_DIR/imgui_draw.cpp" \
    "$$IMGUI_DIR/imgui_tables.cpp" \
    "$$IMGUI_DIR/imgui_widgets.cpp" \
    "$$IMGUI_DIR/backends/imgui_impl_opengl3.cpp" \
    "$$IMPLOT3D_DIR/implot3d.cpp" \
    "$$IMPLOT3D_DIR/implot3d_items.cpp"


DEFINES += IMGUI_IMPL_OPENGL_LOADER_CUSTOM

HEADERS += mainwindow.h ImGuiGLWidget.h \
    xplaneudpreceiver.h
FORMS += mainwindow.ui
DEFINES += QT_DEPRECATED_WARNINGS
