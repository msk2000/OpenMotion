/*
 * ImGuiGLWidget
 * -------------
 * A minimal Qt OpenGL widget that renders a 3D Stewart platform preview using
 * Dear ImGui + ImPlot3D. You push 3×6 matrices (column-major) for base, top,
 * and knee positions; the widget converts them to float arrays and renders:
 *   - Base hex (servo centers)
 *   - Top hex (platform joints)
 *   - Servo arms (base[i] -> knee[i])
 *   - Legs (knee[i] -> top[i]) or a fallback virtual leg if knees absent
 *
 * Axis ranges are computed dynamically from the latest geometry.
 * Please consult with ImPlot3D examples and ImGui examples to understand the conventions used here
 */
#ifndef IMGUIGLWIDGET_H
#define IMGUIGLWIDGET_H

#pragma once

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <vector>


//TODO: Use doxygen style comment formatting like in the setPlatformGeometry universally

// Forward-declare ImGui context type to avoid including <imgui.h> here.
struct ImGuiContext;

class ImGuiGLWidget : public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT
public:
    explicit ImGuiGLWidget(QWidget* parent = nullptr);
    ~ImGuiGLWidget() override;

    /**
     * Push new geometry (expects exactly 6 points for each array).
     * Pointers can be nullptr to skip updating that set this frame.
     *
     * Layout: column-major 3x6 (Eigen default). For column i:
     *   X = M[0 + 3*i],  Y = M[1 + 3*i],  Z = M[2 + 3*i]
     *
     * @param servo_pos_3x6  Base joints (Servo_pos), size 3x6 (double, column-major)
     * @param top_pos_3x6    Platform joints (New_pos), size 3x6 (double, column-major)
     * @param knee_pos_3x6   Elbow joints (Knee_pos_new), size 3x6 (double, column-major)
     */
    void setPlatformGeometry(const double* servo_pos_3x6,
                             const double* top_pos_3x6,
                             const double* knee_pos_3x6);

    void setFixedWorldBounds(float xmin, float xmax,
                             float ymin, float ymax,
                             float zmin, float zmax);
protected:
    // QOpenGLWidget overrides
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;

    // Minimal Qt -> ImGui input bridge
    void mousePressEvent(QMouseEvent*) override;
    void mouseReleaseEvent(QMouseEvent*) override;
    void mouseMoveEvent(QMouseEvent*) override;
    void wheelEvent(QWheelEvent*) override;
    void keyPressEvent(QKeyEvent*) override;
    void keyReleaseEvent(QKeyEvent*) override;
    void focusOutEvent(QFocusEvent*) override;

private:
    // ImGui frame prep and main 3D plot renderer
    void beginNewImGuiFrame_();
    void renderImGui3D_();

    // Compute plot bounds (min/max of all provided points), with a fallback cube
    void computeBounds_(float& xmin, float& xmax,
                        float& ymin, float& ymax,
                        float& zmin, float& zmax) const;


    // Dear ImGui state
    ImGuiContext* imgui_ctx_ = nullptr;

    // Cached geometry (float vectors used by ImPlot3D)
    std::vector<float> x_top_,  y_top_,  z_top_;   // 6 platform joint positions
    std::vector<float> x_base_, y_base_, z_base_;  // 6 base joint positions
    std::vector<float> x_knee_, y_knee_, z_knee_;  // 6 elbow/knee positions

    // Auto-fit bookkeeping
    bool limits_set_ = true;

    // NEW: fixed-bounds state
    bool  use_fixed_limits_ = true;
    float fxmin_ = -160.f, fxmax_ = 160.f;
    float fymin_ = -160.f, fymax_ = 160.f;
    float fzmin_ =  -20.f, fzmax_ = 200.f;
};

#endif // IMGUIGLWIDGET_H
