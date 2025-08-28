/******************************************************************************
 * @file    ImGuiGLWidget.cpp
 * @brief   QOpenGLWidget integration of Dear ImGui and ImPlot3D for rendering
 *          and interacting with a 6-DOF motion platform visualization.
 *
 * This class provides:
 *   - Initialization and shutdown of Dear ImGui + OpenGL3 backend.
 *   - ImPlot3D context for plotting Stewart platform geometry.
 *   - Real-time Qt repaint loop (~60 Hz).
 *   - Geometry ingestion from Eigen 3x6 matrices (servo, top, knee points).
 *   - Automatic or fixed world bounds calculation for plots.
 *   - Rendering of base/top hexagons, servo arms, and legs in 3D.
 *   - Minimal Qt → ImGui input bridge (mouse, wheel, keyboard).
 *
 * Dependencies:
 *   - Qt (QOpenGLWidget, QTimer, QMouseEvent, etc.)
 *   - Dear ImGui (core + OpenGL3 backend)
 *   - ImPlot3D (3D plotting for platform visualization)
 *
 * Part of the Open Motion project (6DOF Motion Simulator).
 *
 * @author  Mahmud Safat Khan
 * @date    2025
 ******************************************************************************/


#include "ImGuiGLWidget.h"

#include <QElapsedTimer>
#include <QMouseEvent>
#include <QKeyEvent>
#include <QWheelEvent>
#include <QTimer>
#include <QDebug>

// Dear ImGui + backend
#include "imgui.h"
#include "backends/imgui_impl_opengl3.h"

// ImPlot3D
#include "implot3d.h"

/**
 * @brief Construct the ImGui-enabled OpenGL widget.
 *
 * Sets strong focus, enables mouse tracking, and starts a ~60 Hz timer
 * to trigger repaints via update().
 *
 * @param parent Optional parent widget (Qt ownership semantics apply).
 */

ImGuiGLWidget::ImGuiGLWidget(QWidget* parent)
    : QOpenGLWidget(parent)
{
    setFocusPolicy(Qt::StrongFocus);
    setMouseTracking(true);

    // Simple heartbeat to repaint ~60 FPS
    auto* timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, QOverload<>::of(&ImGuiGLWidget::update));
    timer->start(16); // ~60 Hz
}

/**
 * @brief Destructor; shuts down ImGui/ImPlot3D and releases GL resources.
 *
 * Ensures the current context is made current, then calls:
 *  - ImGui_ImplOpenGL3_Shutdown()
 *  - ImPlot3D::DestroyContext()
 *  - ImGui::DestroyContext(imgui_ctx_)
 * Finally releases the context with doneCurrent().
 */

ImGuiGLWidget::~ImGuiGLWidget()
{
    makeCurrent();
    ImGui_ImplOpenGL3_Shutdown();
    ImPlot3D::DestroyContext();
    ImGui::DestroyContext(imgui_ctx_);
    doneCurrent();
}

/**
 * @brief Initialize OpenGL state and Dear ImGui/ImPlot3D contexts.
 *
 * Resolves GL functions through Qt, creates an ImGui context, selects a
 * dark style, initializes the OpenGL3 backend with the appropriate GLSL
 * version (ES vs desktop), and creates the ImPlot3D context.
 *
 * @note Must be called by Qt once a valid GL context exists.
 */

void ImGuiGLWidget::initializeGL()
{
    initializeOpenGLFunctions(); // resolve GL symbols via Qt

    IMGUI_CHECKVERSION();
    imgui_ctx_ = ImGui::CreateContext();
    ImGui::StyleColorsDark();

    // Choose GLSL version string depending on OpenGL ES vs desktop
    const bool isGLES = context()->isOpenGLES();
    const char* glsl = isGLES ? "#version 300 es" : "#version 330";
    ImGui_ImplOpenGL3_Init(glsl);

    ImPlot3D::CreateContext();
}

/**
 * @brief Handle widget resize events by updating the GL viewport.
 *
 * @param w New width in pixels.
 * @param h New height in pixels.
 */

void ImGuiGLWidget::resizeGL(int w, int h)
{
    glViewport(0, 0, w, h);
}

/**
 * @brief Begin a new ImGui frame and update timing/display metrics.
 *
 * Computes a stable DeltaTime using QElapsedTimer, clamps outliers
 * (e.g., long pauses) to keep UI interactions responsive, and updates
 * ImGuiIO::DisplaySize to the current widget size.
 *
 * @note Internal helper; not intended to be called outside the render loop.
 */

void ImGuiGLWidget::beginNewImGuiFrame_()
{
    ImGuiIO& io = ImGui::GetIO();

    static QElapsedTimer timer;
    static bool inited = false;
    static qint64 last_ns = 0;

    if (!inited)
    {
        timer.start();
        last_ns = timer.nsecsElapsed();
        inited = true;
    }

    const qint64 now_ns = timer.nsecsElapsed();
    double dt = (now_ns - last_ns) / 1e9;
    last_ns = now_ns;

    // Clamp dt to keep ImGui stable if the timer hiccups
    if (dt <= 0.0 || dt > 0.25)
    {
        dt = 1.0 / 60.0;
    }

    io.DeltaTime   = static_cast<float>(dt);
    io.DisplaySize = ImVec2(float(width()), float(height()));
}

/**
 * @brief Ingest platform geometry from column-major 3x6 arrays (Eigen layout).
 *
 * Copies {x,y,z} rows from each 3x6 matrix (servo/base, top, knee) into
 * internal std::vector<float> containers. If any pointer is null, that set
 * is left unchanged. Triggers bounds recomputation unless fixed limits are set,
 * and requests a repaint.
 *
 * @param servo_pos_3x6 Pointer to 18 doubles: base/servo anchor positions.
 * @param top_pos_3x6   Pointer to 18 doubles: top/platform anchor positions.
 * @param knee_pos_3x6  Pointer to 18 doubles: intermediate “knee” joint positions.
 *
 * @warning Arrays are expected to be Eigen-style column-major: index = row + 3*col.
 * @note Each set should contain exactly 6 points; partial sizes are ignored.
 */


void ImGuiGLWidget::setPlatformGeometry(const double* servo_pos_3x6,
                                        const double* top_pos_3x6,
                                        const double* knee_pos_3x6)
{
    auto fill = [](std::vector<float>& X,
                   std::vector<float>& Y,
                   std::vector<float>& Z,
                   const double* M3x6) {
        if (!M3x6) return;
        X.clear(); Y.clear(); Z.clear();
        X.reserve(6); Y.reserve(6); Z.reserve(6);
        // Eigen default = column-major: idx = row + rows * col
        for (int i = 0; i < 6; ++i)
        {
            X.push_back(static_cast<float>(M3x6[0 + 3 * i])); // row 0 (x), col i
            Y.push_back(static_cast<float>(M3x6[1 + 3 * i])); // row 1 (y), col i
            Z.push_back(static_cast<float>(M3x6[2 + 3 * i])); // row 2 (z), col i
        }
    };

    if (servo_pos_3x6) fill(x_base_, y_base_, z_base_, servo_pos_3x6);
    if (top_pos_3x6)   fill(x_top_,  y_top_,  z_top_,  top_pos_3x6);
    if (knee_pos_3x6)  fill(x_knee_, y_knee_, z_knee_, knee_pos_3x6);

    // New geometry -> recompute bounds on next render
    if (!use_fixed_limits_)
    {
        limits_set_ = false;
    }

    update(); // request repaint
}

/**
 * @brief Compute axis-aligned bounds across available platform geometry.
 *
 * Scans base, top, and knee point sets and writes min/max for each axis.
 * If no geometry is present, provides a sane default volume.
 *
 * @param[out] xmin Minimum x.
 * @param[out] xmax Maximum x.
 * @param[out] ymin Minimum y.
 * @param[out] ymax Maximum y.
 * @param[out] zmin Minimum z.
 * @param[out] zmax Maximum z.
 *
 * @note Internal helper for auto-fit behavior.
 */


void ImGuiGLWidget::computeBounds_(float& xmin, float& xmax,
                                   float& ymin, float& ymax,
                                   float& zmin, float& zmax) const
{
    auto scan = [&](const std::vector<float>& X,
                    const std::vector<float>& Y,
                    const std::vector<float>& Z)
    {
        const size_t n = std::min({X.size(), Y.size(), Z.size()});
        for (size_t i = 0; i < n; ++i)
        {
            xmin = std::min(xmin, X[i]); xmax = std::max(xmax, X[i]);
            ymin = std::min(ymin, Y[i]); ymax = std::max(ymax, Y[i]);
            zmin = std::min(zmin, Z[i]); zmax = std::max(zmax, Z[i]);
        }
    };

    xmin = ymin = zmin =  std::numeric_limits<float>::infinity();
    xmax = ymax = zmax = -std::numeric_limits<float>::infinity();

    scan(x_base_, y_base_, z_base_);
    scan(x_top_,  y_top_,  z_top_);
    scan(x_knee_, y_knee_, z_knee_);

    // If nothing is present yet, provide a sane default volume
    if (!std::isfinite(xmin))
    {
        xmin = ymin = -150.f; xmax = ymax = 150.f;
        zmin = -50.f;         zmax = 200.f;
    }
}

/**
 * @brief Render the 3D ImGui/ImPlot3D view of the Stewart platform.
 *
 * Creates a full-window ImGui panel, begins an ImPlot3D plot, applies either
 * fixed limits or a one-shot auto-fit from current geometry, draws reference
 * axes, and plots:
 *  - Closed polygons for base/top hexes (with scatter markers),
 *  - Servo arms (base -> knee),
 *  - Legs (knee -> top), or virtual legs (base -> top) if knees are absent.
 *
 * @note Internal helper invoked from paintGL() between NewFrame() and Render().
 */


void ImGuiGLWidget::renderImGui3D_()
{
    ImGuiIO& io = ImGui::GetIO();
    ImGui::SetNextWindowPos(ImVec2(0, 0));
    ImGui::SetNextWindowSize(io.DisplaySize);

    const ImGuiWindowFlags wf = ImGuiWindowFlags_NoDecoration
                                | ImGuiWindowFlags_NoMove
                                | ImGuiWindowFlags_NoSavedSettings
                                | ImGuiWindowFlags_NoBringToFrontOnFocus;

    ImGui::Begin("Platform View", nullptr, wf);


        // Set/update limits from current geometry (once per update)
        if (ImPlot3D::BeginPlot("6-DOF Platform", ImVec2(-1, -1), 0))
        {

            if (use_fixed_limits_)
            {
                // Always the same world box; grid stays put
                ImPlot3D::SetupAxesLimits(fxmin_, fxmax_,
                                          fymin_, fymax_,
                                          fzmin_, fzmax_,
                                          ImPlot3DCond_Always);
            }
            else if (!limits_set_)
            {
                // Auto-fit once after new geometry arrives
                float xmin, xmax, ymin, ymax, zmin, zmax;
                computeBounds_(xmin, xmax, ymin, ymax, zmin, zmax);
                const float px = 0.10f * std::max(10.f, xmax - xmin);
                const float py = 0.10f * std::max(10.f, ymax - ymin);
                const float pz = 0.10f * std::max(10.f, zmax - zmin);
                ImPlot3D::SetupAxesLimits(xmin - px, xmax + px,
                                          ymin - py, ymax + py,
                                          zmin - pz, zmax + pz,
                                          ImPlot3DCond_Always);
                limits_set_ = true;
            }

        // Optional: origin gizmo for orientation sanity
        {
            float ox[2] = {0.f, 40.f}, oy[2] = {0.f, 0.f}, oz[2] = {0.f, 0.f};
            ImPlot3D::PlotLine("X axis", ox, oy, oz, 2);
            float px[2] = {0.f, 0.f}, py[2] = {0.f, 40.f};
            ImPlot3D::PlotLine("Y axis", px, py, oz, 2);
            float qx[2] = {0.f, 0.f}, qy[2] = {0.f, 0.f}, qz[2] = {0.f, 40.f};
            ImPlot3D::PlotLine("Z axis", qx, qy, qz, 2);
        }

        // Convenience lambda: draw closed polygon + its vertices
        auto plotClosed = [](const char* label,
                             const std::vector<float>& X,
                             const std::vector<float>& Y,
                             const std::vector<float>& Z)
        {
            if (X.size() < 6) return;
            std::vector<float> xc = X, yc = Y, zc = Z;
            xc.push_back(X.front()); yc.push_back(Y.front()); zc.push_back(Z.front());
            ImPlot3D::PlotLine(label, xc.data(), yc.data(), zc.data(), (int)xc.size());
            std::string pts = std::string(label) + " joints";
            ImPlot3D::PlotScatter(pts.c_str(), X.data(), Y.data(), Z.data(), 6);
        };

        // Draw top and base hexes
        if (x_top_.size()  == 6) plotClosed("Top Hex",  x_top_,  y_top_,  z_top_);
        if (x_base_.size() == 6) plotClosed("Base Hex", x_base_, y_base_, z_base_);

        // Draw servo arms: base[i] -> knee[i]
        if (x_base_.size() == 6 && x_knee_.size() == 6)
        {
            for (int i = 0; i < 6; ++i)
            {
                float ax[2] = { x_base_[i], x_knee_[i] };
                float ay[2] = { y_base_[i], y_knee_[i] };
                float az[2] = { z_base_[i], z_knee_[i] };
                std::string lbl = "Arm " + std::to_string(i);
                ImPlot3D::PlotLine(lbl.c_str(), ax, ay, az, 2);
            }
        }

        // Draw legs: knee[i] -> top[i], with fallback if knees not set yet
        if (x_knee_.size() == 6 && x_top_.size() == 6)
        {
            for (int i = 0; i < 6; ++i)
            {
                float lx[2] = { x_knee_[i], x_top_[i] };
                float ly[2] = { y_knee_[i], y_top_[i] };
                float lz[2] = { z_knee_[i], z_top_[i] };
                std::string lbl = "Leg " + std::to_string(i);
                ImPlot3D::PlotLine(lbl.c_str(), lx, ly, lz, 2);
            }
        }
        else if (x_base_.size() == 6 && x_top_.size() == 6)
        {
            for (int i = 0; i < 6; ++i)
            {
                float vx[2] = { x_base_[i], x_top_[i] };
                float vy[2] = { y_base_[i], y_top_[i] };
                float vz[2] = { z_base_[i], z_top_[i] };
                std::string lbl = "Virtual Leg " + std::to_string(i);
                ImPlot3D::PlotLine(lbl.c_str(), vx, vy, vz, 2);
            }
        }

        ImPlot3D::EndPlot();
    }

    ImGui::End();
}

/**
 * @brief Main GL paint routine (Qt override).
 *
 * Clears color/depth buffers, advances the ImGui frame, renders the 3D view,
 * and submits ImGui draw data via the OpenGL3 backend.
 */


void ImGuiGLWidget::paintGL()
{
    glEnable(GL_DEPTH_TEST);
    glClearColor(0.10f, 0.11f, 0.12f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    beginNewImGuiFrame_();
    ImGui_ImplOpenGL3_NewFrame();
    ImGui::NewFrame();

    renderImGui3D_();

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

/**
 * @brief Translate Qt mouse press events to ImGui input.
 *
 * Maps Qt buttons to ImGui mouse buttons and updates mouse position.
 *
 * @param e Qt mouse event.
 */


void ImGuiGLWidget::mousePressEvent(QMouseEvent* e)
{
    ImGuiIO& io = ImGui::GetIO();
    if (e->button() == Qt::LeftButton)   io.AddMouseButtonEvent(0, true);
    if (e->button() == Qt::RightButton)  io.AddMouseButtonEvent(1, true);
    if (e->button() == Qt::MiddleButton) io.AddMouseButtonEvent(2, true);
    io.AddMousePosEvent(e->position().x(), e->position().y());
    update();
}

/**
 * @brief Translate Qt mouse release events to ImGui input.
 *
 * Maps Qt buttons to ImGui mouse buttons.
 *
 * @param e Qt mouse event.
 */

void ImGuiGLWidget::mouseReleaseEvent(QMouseEvent* e)
{
    ImGuiIO& io = ImGui::GetIO();
    if (e->button() == Qt::LeftButton)   io.AddMouseButtonEvent(0, false);
    if (e->button() == Qt::RightButton)  io.AddMouseButtonEvent(1, false);
    if (e->button() == Qt::MiddleButton) io.AddMouseButtonEvent(2, false);
    update();
}

/**
 * @brief Translate Qt mouse move events to ImGui input.
 *
 * Updates ImGui mouse position with the event’s local coordinates.
 *
 * @param e Qt mouse event.
 */

void ImGuiGLWidget::mouseMoveEvent(QMouseEvent* e)
{
    ImGui::GetIO().AddMousePosEvent(e->position().x(), e->position().y());
    update();
}

/**
 * @brief Translate Qt wheel events to ImGui input.
 *
 * Converts angleDelta() to ImGui horizontal/vertical wheel deltas.
 *
 * @param e Qt wheel event.
 */

void ImGuiGLWidget::wheelEvent(QWheelEvent* e)
{
    const QPointF d = e->angleDelta() / 120.0;
    ImGui::GetIO().AddMouseWheelEvent(float(d.x()), float(d.y()));
    update();
}

/**
 * @brief Translate Qt key press events to ImGui input.
 *
 * Updates ImGui modifier key states and forwards UTF-8 text input.
 *
 * @param e Qt key event.
 */

void ImGuiGLWidget::keyPressEvent(QKeyEvent* e) {
    ImGuiIO& io = ImGui::GetIO();
    io.AddKeyEvent(ImGuiKey_ModCtrl,  e->modifiers() & Qt::ControlModifier);
    io.AddKeyEvent(ImGuiKey_ModShift, e->modifiers() & Qt::ShiftModifier);
    io.AddKeyEvent(ImGuiKey_ModAlt,   e->modifiers() & Qt::AltModifier);
    io.AddKeyEvent(ImGuiKey_ModSuper, e->modifiers() & Qt::MetaModifier);
    if (!e->text().isEmpty())
        io.AddInputCharactersUTF8(e->text().toUtf8().constData());
    update();
}

/**
 * @brief Translate Qt key release events to ImGui input.
 *
 * Updates ImGui modifier key states.
 *
 * @param e Qt key event.
 */


void ImGuiGLWidget::keyReleaseEvent(QKeyEvent* e)
{
    ImGuiIO& io = ImGui::GetIO();
    io.AddKeyEvent(ImGuiKey_ModCtrl,  e->modifiers() & Qt::ControlModifier);
    io.AddKeyEvent(ImGuiKey_ModShift, e->modifiers() & Qt::ShiftModifier);
    io.AddKeyEvent(ImGuiKey_ModAlt,   e->modifiers() & Qt::AltModifier);
    io.AddKeyEvent(ImGuiKey_ModSuper, e->modifiers() & Qt::MetaModifier);
    update();
}

/**
 * @brief Ensure mouse buttons are released when the widget loses focus.
 *
 * Prevents “stuck button” states in ImGui when focus changes.
 */

void ImGuiGLWidget::focusOutEvent(QFocusEvent*)
{
    ImGuiIO& io = ImGui::GetIO();
    io.AddMouseButtonEvent(0, false);
    io.AddMouseButtonEvent(1, false);
    io.AddMouseButtonEvent(2, false);
}

/**
 * @brief Set fixed world bounds for the 3D plot and disable auto-fit.
 *
 * Subsequent frames will use these limits unconditionally.
 *
 * @param xmin Min X.
 * @param xmax Max X.
 * @param ymin Min Y.
 * @param ymax Max Y.
 * @param zmin Min Z.
 * @param zmax Max Z.
 */

void ImGuiGLWidget::setFixedWorldBounds(float xmin,float xmax,float ymin,float ymax,float zmin,float zmax)
{
    use_fixed_limits_ = true;
    fxmin_=xmin; fxmax_=xmax; fymin_=ymin; fymax_=ymax; fzmin_=zmin; fzmax_=zmax;
}
