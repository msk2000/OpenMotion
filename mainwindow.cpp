/******************************************************************************
 * @file    mainwindow.cpp
 * @brief   Main application window driving the 6-DOF platform UI, geometry,
 *          inverse kinematics, X-Plane UDP ingestion, and Arduino serial I/O.
 *
 * Responsibilities:
 *   - Manage manual/sim drive modes and motion cue gains/limits.
 *   - Compute platform pose → joint geometry → servo angles (IK).
 *   - Stream angles to Arduino over serial at ~60 Hz.
 *   - Render geometry via ImGuiGLWidget/ImPlot3D.
 *   - Show live telemetry dock fed by X-Plane UDP samples.
 *
 * Part of the Open Motion project (6DOF Motion Simulator).
 ******************************************************************************/

#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <iostream>
#include <Eigen/Dense>
#include <math.h>
#include <array>
#include <QDebug>
#include <cstring> // for memset
#include "ImGuiGLWidget.h"
#include "xplaneudpreceiver.h"
#include "config.h"
#include <QTimer>
#include <QDateTime>

static constexpr double RAD2DEG = 180.00 / M_PI;
static constexpr double DEG2RAD = M_PI / 180.0;
//For geometry
// Angles


// Excel geometry (all in mm / degrees)
constexpr double P_RAD   = 49.805;              // Platform radius RD
constexpr double B_RAD   = 77.0;                // Base radius    PD
constexpr double L1 = 37.0;          // L1
constexpr double L2 = 140.0;       // L2 (pushrod)
constexpr double Z_HOME = 135.0;                // default Z height (arms horizontal)

// Azimuth offsets (from Excel):
//  - Theta_R: platform joint angle offset
//  - Theta_P: base (servo pinion) angle offset
constexpr double THETA_R = 22.9492 * DEG2RAD;   // platform joint offset
constexpr double THETA_P = 48.4099 * DEG2RAD;   // base servo offset

// Global base rotation (0 = flat-top). Set to 30*DEG2RAD if base is "pointy-top".
constexpr double GAMMA = 30.0 * DEG2RAD;

// Servo horn axis directions (Theta_S, degrees -> radians)
constexpr std::array<double,6> kBeta = {
    -60.0*DEG2RAD,  120.0*DEG2RAD,  180.0*DEG2RAD,
    0.0*DEG2RAD,   60.0*DEG2RAD, -120.0*DEG2RAD
};



/**
 * @brief Wrap an angle in degrees to (-180, 180].
 *
 * @param a Angle in degrees.
 * @return Wrapped angle in degrees.
 */

static inline double wrapDeg(double a)
{
    while (a > 180.0) a -= 360.0;
    while (a < -180.0) a += 360.0;
    return a;
}
/**
 * @brief Clamp a value to [lo, hi].
 *
 * @param v  Input value.
 * @param lo Lower bound (inclusive).
 * @param hi Upper bound (inclusive).
 * @return Clamped value.
 */

static inline double clampD(double v, double lo, double hi)
{
    return std::max(lo, std::min(hi, v));
}

/**
 * @brief Construct the main window and initialize subsystems.
 *
 * Sets up UI, UDP receiver (X-Plane), 60 Hz tick, serial port retry timer,
 * serial connection, telemetry dock, and 3D plot world bounds. Wires slider
 * signals, initializes geometry from platform angles, computes initial pose,
 * and primes translation vectors.
 *
 * @param parent Optional parent widget.
 */


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    // Initialize all member variables in the initializer list

        X(0),
        Y(0),
        Z(0),
        phi(0),
        theta(0),
        psi(0),
        angles{90.0, 90.0, 90.0, 90.0, 90.0, 90.0}, // Initialize with zeros or any default value
        servo_arm(L1),
        servo_leg(L2),
        beta{ kBeta[0], kBeta[1], kBeta[2], kBeta[3], kBeta[4], kBeta[5] },//{0, -M_PI / 3, -2 * M_PI / 3, -M_PI, -4 * M_PI / 3, -5 * M_PI / 3},        //beta{M_PI / 2, M_PI / 6, -M_PI/6,  -M_PI / 2, -5 * M_PI / 6, -2* M_PI /3},//beta{M_PI / 3, -2 * M_PI / 3, M_PI, 0, 5 * M_PI / 3, 2 * M_PI / 3},
        height(115) // Initialization of height

{

    // Make sure the sliders are all connected
    ui->setupUi(this);

    //================== X-Plane reciever initialisations =================
    udpRx_ = new XPlaneUdpReceiver(this);
    if(!udpRx_->start(XP_UDP_REC_PORT))
    {
            qCritical() << "UDP binding crashed and burnt lol";
    }

    /*connect(udpRx_, &XPlaneUdpReceiver::sampleReady,this,[this](const MotionSample& s)
    {
        // Test: drive 6‑DOF target state from X‑Plane
        // Pushing angles directly, position stays home + small heave cue
        phi   = s.roll_deg  * DEG2RAD;
        theta = s.pitch_deg * DEG2RAD;
        psi   = s.yaw_deg   * DEG2RAD;

        // TODO: do motion cue scaling here (washout, gains, limits)
        // For now, mayb just recompute geometry & send to Arduino at a controlled rate
        stepPlatformAndSerial();
        // TODO: And update GUI labels :
        // ui->labelRoll->setText(QString::number(s.roll_deg, 'f', 2));
    });*/

    // Rate‑limit platform updates (e.g., 60 Hz)
    tick_ = new QTimer(this);
    connect(tick_, &QTimer::timeout, this, &MainWindow::stepPlatformAndSerial);
    tick_->start(16); // ~60Hz


    //  retry timer if the board gets unplugged and replugged
    serialRetry_ = new QTimer(this);
    serialRetry_->setInterval(2000); // try every 2s when not connected
    connect(serialRetry_, &QTimer::timeout, this, &MainWindow::retryOpenSerialQt);

    // Open serial once
    setupSerialQt();
    // If not connected yet, start retrying
    if (!sp_ || !sp_->isOpen()) serialRetry_->start();
    //======================================================================

    //============ Telemetry Widget ====================================
    createTelemetryDock_();

    connect(udpRx_, &XPlaneUdpReceiver::sampleReady, this, [this](const MotionSample& s)
    {
        // Update labels (2 decimals)
        if (lblRoll_)  lblRoll_->setText(QString::number(s.roll_deg,  'f', 2));
        if (lblPitch_) lblPitch_->setText(QString::number(s.pitch_deg, 'f', 2));
        if (lblYaw_)   lblYaw_->setText(QString::number(s.yaw_deg,   'f', 2));

        if (lblP_) lblP_->setText(QString::number(s.p, 'f', 3));
        if (lblQ_) lblQ_->setText(QString::number(s.q, 'f', 3));
        if (lblR_) lblR_->setText(QString::number(s.r, 'f', 3));

        if (lblAx_) lblAx_->setText(QString::number(s.ax, 'f', 2));
        if (lblAy_) lblAy_->setText(QString::number(s.ay, 'f', 2));
        if (lblAz_) lblAz_->setText(QString::number(s.az, 'f', 2));

        // Cache for "Home" and watchdog
        lastSample_ = s;
        lastHaveSample_ = true;
        lastSimMs_ = QDateTime::currentMSecsSinceEpoch();

        if (driveMode_ != DriveMode::FromSim || !haveSimZero_)
        {
            return; // either manual, or we haven't zeroed to sim yet
        }

        // Relative angles (deg) w/ wrapped yaw
        const double relRollDeg  = s.roll_deg  - roll0_deg;
        const double relPitchDeg = s.pitch_deg - pitch0_deg;
        const double relYawDeg   = wrapDeg(s.yaw_deg - yaw0_deg);

        // Gains + clamp, willb convert to radians for math when needed
        // use axis_ signs to fix roll/pitch/yaw sense in one place
        phi   = clampD(axis_.roll  * (relRollDeg  * cue_.kRoll ) * DEG2RAD, -cue_.maxRoll,  cue_.maxRoll);
        theta = clampD(axis_.pitch * (relPitchDeg * cue_.kPitch) * DEG2RAD, -cue_.maxPitch, cue_.maxPitch);
        psi   = clampD(axis_.yaw   * (relYawDeg   * cue_.kYaw  ) * DEG2RAD, -cue_.maxYaw,   cue_.maxYaw);


        // Timer will render/send at ~60 Hz
    });




    // get the promoted widget by objectName
    glw_ = this->findChild<ImGuiGLWidget*>("plot3D");  // must match Designer objectName
    if (!glw_)
    {
        qCritical() << "ImGuiGLWidget not found. Did I even promote the widget and set objectName to 'plot3D'?";
    }
    // conservative static box; tweak as you like
    glw_->setFixedWorldBounds(-100.f, 100.f, -100.f, 100.f, -20.f, 190.f);

    connect(ui->Slider_X, SIGNAL(valueChanged(int)), this, SLOT(on_Slider_X_valueChanged(int)));
    connect(ui->Slider_Y, SIGNAL(valueChanged(int)), this, SLOT(on_Slider_Y_valueChanged(int)));
    connect(ui->Slider_Z, SIGNAL(valueChanged(int)), this, SLOT(on_Slider_Z_valueChanged(int)));
    connect(ui->Slider_PHI, SIGNAL(valueChanged(int)), this, SLOT(on_Slider_PHI_valueChanged(int)));
    connect(ui->Slider_THETA, SIGNAL(valueChanged(int)), this, SLOT(on_Slider_THETA_valueChanged(int)));
    connect(ui->Slider_PSI, SIGNAL(valueChanged(int)), this, SLOT(on_Slider_PSI_valueChanged(int)));



    //======================Platform Geometry ====================\\
    /*
     //Initialise platform related constants here

    // Platform points (top) Pi
    Platform_pos_zero << -41.4, -53.9, -12.5, 12.5, 53.9, 41.4, // First row
                         38.3, 16.7, -55, -55, 16.7, 38.3,      // Second row
                         0, 0, 0, 0, 0, 0;                     // Third row



    // Servo points (base) //! z changed to all zeros // Trying to make it sy
    Servo_pos << -46.5, -90.5, -43.5, 43.5, 90.5, 46.5,  // First row
                  74, 1.1, -81, -81, 1.14, 74,           // Second row
                  0, 0, 0, 0, 0, 0;                     // Third row */


    initGeometryFromAngles_();
    // Rotation matrix (R = Rz*Ry*Rx) used to take platform stuff to base frame

    R_PB << cos(psi) * cos(theta),
            (-sin(psi) * cos(phi)) + (cos(psi) * sin(theta) * sin(phi)),
            (sin(psi) * sin(phi)) + (cos(psi) * sin(theta) * cos(phi)),

            sin(psi) * cos(theta),
            (cos(psi) * cos(phi)) + (sin(psi) * sin(theta) * sin(phi)),
            (-cos(psi) * sin(phi)) + (sin(psi) * sin(theta) * cos(phi)),

            -sin(theta),
            cos(theta) * sin(phi),
            cos(theta) * cos(phi);

    // Height of the platform when servo arm is perpendicular to leg
    h_0 = sqrt(std::pow(servo_leg,2) + std::pow(servo_arm,2)
            - std::pow(Platform_pos_zero(0,0) - Servo_pos(0,0),2)
            - std::pow(Platform_pos_zero(1,0) - Servo_pos(1,0),2))
            - Platform_pos_zero(2,0);

    // The translation values for default home position
    t_home << 0, 0, h_0;
    // Translation values with input
    t_input << X, Y, Z;
    // Total translation
    T = t_home + t_input;


}

/**
 * @brief Destructor; closes serial and releases UI resources.
 */

MainWindow::~MainWindow()
{
    closeSerialQt();
    delete ui;


}

/**
 * @brief Home/zero action for platform pose or sim-relative origin.
 *
 * If driving from sim, captures current sim Euler angles as zero reference
 * (roll0_deg, pitch0_deg, yaw0_deg) and sets haveSimZero_. In all cases,
 * resets platform attitude to zero and recenters translation (X,Y only).
 * Geometry/serial updates are handled by the 60 Hz tick.
 */

void MainWindow::on_homeButton_clicked()
{
    if (driveMode_ == DriveMode::FromSim)
    {
        // Use current sim as the origin and reset platform pose to zero.
        if (lastHaveSample_)
        {
            roll0_deg  = lastSample_.roll_deg;
            pitch0_deg = lastSample_.pitch_deg;
            yaw0_deg   = lastSample_.yaw_deg;
            haveSimZero_ = true;
        }
        // Zero platform attitude and recentre translation
        phi = theta = psi = 0.0;
        X = Y = 0.0; // optional: keep translational home
        t_input << X, Y, Z;
        T = t_home + t_input;
        // Let the 60 Hz tick recompute geometry & send
        return;
    }
    // The current use of this is to reset the platform to the zero position and update the parameters accordingly

    // Set all sliders to zero position
       ui ->Slider_X->setValue(0);
       ui ->Slider_Y->setValue(0);
       ui ->Slider_Z->setValue(0);

       ui ->Slider_PHI->setValue(0);
       ui ->Slider_THETA->setValue(0);
       ui ->Slider_PSI->setValue(0);



       //1. Calculate platform's home position (New_pos)
       Rotated_platform = R_PB * Platform_pos_zero;

       New_pos = T.replicate<1, 6>().array() + Rotated_platform.array();


       //2. Calculate angle of the servo arm at home position
       // First find the linear Leg length

       lin_leg_lengths = New_pos - Servo_pos;

       // The .colwise().norm() method calculates the Euclidean norm of each column,
       // which corresponds to the length of each leg vector
       virtual_leg_lengths = (lin_leg_lengths).colwise().norm();




       // Due to platform symmetry, we can only consider the leg with 0 beta //!Are we symmetric?
       L_home = 2 * std::pow(servo_arm, 2);
       M_home = 2 * servo_arm * (New_pos(0,0) - Servo_pos(0,0));
       N_home = 2 * servo_arm * (h_0 + New_pos(2,3));

       alpha_home = asin(L_home/sqrt(pow(M_home,2)+pow(N_home,2))) - atan(M_home/N_home);
       alpha_home_deg = RAD2DEG * alpha_home;


       // ======== workout the servo arm/leg join positions ====


       // Changed knee position


       for (size_t j=0; j<6; j++)
        {
            Knee_pos_home(0,j) = servo_arm*cos(alpha_home)*cos(beta[j]) + Servo_pos(0,j);
            Knee_pos_home(1,j) = servo_arm*cos(alpha_home)*sin(beta[j]) + Servo_pos(1,j);
            Knee_pos_home(2,j) = servo_arm*sin(alpha_home) + Servo_pos(2,j);
        }





        pushGeometryToGL();


}

/**
 * @brief Clamp helper (double) with inclusive range.
 *
 * @param value Input value.
 * @param min   Lower bound (inclusive).
 * @param max   Upper bound (inclusive).
 * @return Clamped value.
 */


double MainWindow::clamp(double value, double min, double max)
{
    if (value < min)
    {
        return min;
    }
    if (value > max)
    {
        return max;
    }
        return value;
}

/**
 * @brief One-shot recompute of platform geometry and servo angles; send to MCU.
 *
 * Rebuilds rotation matrix from (phi,theta,psi), transforms platform points,
 * computes leg vectors and lengths, derives servo angles (alpha) via IK
 * (using L, M, N terms), maps to logical degrees in [0,180], formats a CSV
 * angle line, writes to serial, updates knee joint positions, and pushes the
 * new geometry to the GL widget.
 *
 * @note Typically invoked by sliders; the periodic path is stepPlatformAndSerial().
 */

void MainWindow::on_startButton_clicked()
{



           // 1. Update the rotation matrix based on current phi theta psi
           // Rotation matrix (R = Rz*Ry*Rx) used to take platform stuff to base frame
           R_PB << cos(psi) * cos(theta),
                   (-sin(psi) * cos(phi)) + (cos(psi) * sin(theta) * sin(phi)),
                   (sin(psi) * sin(phi)) + (cos(psi) * sin(theta) * cos(phi)),

                   sin(psi) * cos(theta),
                   (cos(psi) * cos(phi)) + (sin(psi) * sin(theta) * sin(phi)),
                   (-cos(psi) * sin(phi)) + (sin(psi) * sin(theta) * cos(phi)),

                   -sin(theta),
                   cos(theta) * sin(phi),
                   cos(theta) * cos(phi);

           // 2. Calculate platform's transformed position(New_pos)
           Rotated_platform = R_PB * Platform_pos_zero;


           New_pos = T.replicate<1, 6>().array() + Rotated_platform.array();


           //3.  Calculate angle of the servo arm at NEW position
           // First find the linear Leg length

           lin_leg_lengths = New_pos - Servo_pos;

           // The .colwise().norm() method calculates the Euclidean norm of each column,
           // which corresponds to the length of each leg vector
           virtual_leg_lengths = (lin_leg_lengths).colwise().norm();


           // Calculate the servo angles for each leg


            for (size_t i=0;i<6;i++)
            {

                L(0,i) = std::pow(virtual_leg_lengths(0,i),2)
                         - ((std::pow(servo_leg,2)) - (std::pow(servo_arm,2)));

            }




            for (size_t i=0;i<6;i++)
            {
                M(0,i) = 2*servo_arm*(New_pos(2,i)-Servo_pos(2,i));

            }




            for (size_t i=0;i<6;i++)
            {
                x_diff  = New_pos(0,i) - Servo_pos(0,i); // intermediate calc
                y_diff  = New_pos(1,i) - Servo_pos(1,i);
                N(0,i) = 2*servo_arm*((cos(beta[i])*x_diff)+(sin(beta[i])*y_diff));
            }



            // Now we can calculate the servo angles

            for (size_t i = 0; i <6; i++)
            {

                alpha(0,i) = std::asin(L(0,i) / std::sqrt(std::pow(M(0,i),2) + std::pow(N(0,i),2)))
                - std::atan2(N(0,i), M(0,i));

                // Map: alpha = 0 (horizontal)  -> send 90
                //      alpha < 0 (below)       -> send < 90
                //      alpha > 0 (above)       -> send > 90
                const double logical_deg = 90.0 + (RAD2DEG * alpha[i]);

                // Safety: only clamp AFTER adding the offset, so negative alpha survives
                angles[i] = clampD(logical_deg, 0.0, 180.0);


            }

            //4.  Convert angles to a string for sending over serial

            std::ostringstream angleStream;
            for (size_t i = 0; i < angles.size(); ++i)
            {
                angleStream << static_cast<int>(angles[i]); // Convert to int
                if (i < angles.size() - 1)
                {
                    angleStream << ","; // Add delimiter
                }
            }
            angleStream << "\n"; // End the string with a newline

            // Send the angle string to Arduino using custom serial method
            sendAnglesQt(QString::fromStdString(angleStream.str()));




         //Changed knee position}


            for (size_t j=0; j<6; j++)
            {
                Knee_pos_new(0,j) = servo_arm*cos(alpha[j])*cos(beta[j]) + Servo_pos(0,j);
                Knee_pos_new(1,j) = servo_arm*cos(alpha[j])*sin(beta[j]) + Servo_pos(1,j);
                Knee_pos_new(2,j) = servo_arm*sin(alpha[j]) + Servo_pos(2,j);
            }



    pushGeometryToGL(); // Hope to god this works!




}


/**
 * @brief Abort/quit the application.
 */

void MainWindow::on_abortButton_clicked()
{
    QApplication::quit();
}

/**
 * @brief Slider callback for X translation.
 *
 * Scales and clamps by cue settings, updates translation vector T, and
 * triggers a geometry/serial recompute.
 *
 * @param value Slider value (int).
 */

void MainWindow::on_Slider_X_valueChanged(int value)
{
    X = clampD(cue_.kX * value, -cue_.maxX,  cue_.maxX);
    t_input << X, Y, Z;
    T = t_home + t_input;
    on_startButton_clicked();
}
/**
 * @brief Slider callback for Y translation.
 *
 * @param value Slider value (int).
 */

void MainWindow::on_Slider_Y_valueChanged(int value)
{
    Y = clampD(cue_.kY * value, -cue_.maxY,  cue_.maxY);
    t_input << X, Y, Z;
    T = t_home + t_input;
    on_startButton_clicked();
}
/**
 * @brief Slider callback for Z translation (heave).
 *
 * @param value Slider value (int).
 */

void MainWindow::on_Slider_Z_valueChanged(int value)
{
    Z = clampD(cue_.kZ * value, -cue_.maxZ,  cue_.maxZ);
    t_input << X, Y, Z;
    T = t_home + t_input;
    on_startButton_clicked();
}

/**
 * @brief Slider callback for roll (phi) in degrees.
 *
 * Applies axis sign, converts to radians, updates phi, and recomputes.
 *
 * @param value Slider value (deg).
 */

void MainWindow::on_Slider_PHI_valueChanged(int value)
{
    // value is degrees from the slider
    setPHI(axis_.roll * DEG2RAD * value);
    on_startButton_clicked();
}

/**
 * @brief Slider callback for pitch (theta) in degrees.
 *
 * @param value Slider value (deg).
 */

void MainWindow::on_Slider_THETA_valueChanged(int value)
{
    setTHETA(axis_.pitch * DEG2RAD * value);
    on_startButton_clicked();
}

/**
 * @brief Slider callback for yaw (psi) in degrees.
 *
 * @param value Slider value (deg).
 */

void MainWindow::on_Slider_PSI_valueChanged(int value)
{
    setPSI(axis_.yaw * DEG2RAD * value);
    on_startButton_clicked();
}

/**
 * @brief Set X translation (meters or UI units per cue).
 *
 * @param value New X.
 */

void MainWindow::setX(double value)
{
    X = value;
}
/**
 * @brief Set Y translation.
 *
 * @param value New Y.
 */

void MainWindow::setY(double value)
{
    Y = value;
}
/**
 * @brief Set Z translation.
 *
 * @param value New Z.
 */

void MainWindow::setZ(double value)
{
    Z = value;
}


/**
 * @brief Set roll angle phi (radians).
 *
 * @param value phi in radians.
 */

void MainWindow::setPHI(double value)
{
    phi = value;
}
/**
 * @brief Set pitch angle theta (radians).
 *
 * @param value theta in radians.
 */

void MainWindow::setTHETA(double value)
{
    theta = value;
}

/**
 * @brief Set yaw angle psi (radians).
 *
 * @param value psi in radians.
 */

void MainWindow::setPSI(double value)
{
    psi = value;
}

/**
 * @brief Push current platform geometry to the GL renderer.
 *
 * Sends base (Servo_pos), transformed platform (New_pos), and knee (Knee_pos_new)
 * 3×6 matrices to ImGuiGLWidget for visualization. No-ops if widget not present.
 */

void MainWindow::pushGeometryToGL()
{
    if (!glw_) return;
    glw_->setPlatformGeometry(Servo_pos.data(), New_pos.data(), Knee_pos_new.data());
}

/**
 * @brief Periodic (≈60 Hz) update: pose decay, IK, serial send, and GL update.
 *
 * - If driving from sim and packets stale > 0.5 s, exponentially decay
 *   (phi,theta,psi) toward zero (washout-like) to avoid stale poses.
 * - Recompute rotation matrix and transformed platform points.
 * - Perform IK to compute servo angles, clamp to [0,180], and send CSV line
 *   to Arduino if the serial port is open.
 * - Update knee joint positions and push geometry to the renderer.
 */


void MainWindow::stepPlatformAndSerial()
{
    //== Return to origin in case sim stops ===
    const qint64 nowMs = QDateTime::currentMSecsSinceEpoch();
    if (driveMode_ == DriveMode::FromSim)
    {
        const bool stale = (nowMs - lastSimMs_) > 500; // no packets for 0.5s
        if (stale)
        {
            // exponential decay to zero over ~0.6 s
            constexpr double dt  = 1.0/60.0;
            constexpr double tau = 0.6;
            const double a = std::exp(-dt/tau);
            phi   *= a;
            theta *= a;
            psi   *= a;
        }
    }

    // 1) Recompute R_PB from current phi/theta/psi (these now come from either sliders or UDP)
    R_PB << cos(psi) * cos(theta),
        (-sin(psi) * cos(phi)) + (cos(psi) * sin(theta) * sin(phi)),
        (sin(psi) * sin(phi)) + (cos(psi) * sin(theta) * cos(phi)),
        sin(psi) * cos(theta),
        (cos(psi) * cos(phi)) + (sin(psi) * sin(theta) * sin(phi)),
        (-cos(psi) * sin(phi)) + (sin(psi) * sin(theta) * cos(phi)),
        -sin(theta),
        cos(theta) * sin(phi),
        cos(theta) * cos(phi);

    // 2) New positions
    Rotated_platform = R_PB * Platform_pos_zero;
    New_pos = T.replicate<1,6>().array() + Rotated_platform.array();

    // 3) Perform the Inverse Kinematics to derive the servo angles
    lin_leg_lengths = New_pos - Servo_pos;
    virtual_leg_lengths = lin_leg_lengths.colwise().norm();

    for (size_t i=0;i<6;i++)
    {
        L(0,i) = std::pow(virtual_leg_lengths(0,i),2) - (std::pow(servo_leg,2) - std::pow(servo_arm,2));
        M(0,i) = 2*servo_arm*(New_pos(2,i)-Servo_pos(2,i));
        double x_diff = New_pos(0,i) - Servo_pos(0,i);
        double y_diff = New_pos(1,i) - Servo_pos(1,i);
        N(0,i) = 2*servo_arm*(cos(beta[i])*x_diff + sin(beta[i])*y_diff);

        alpha(0,i) = std::asin(L(0,i) / std::sqrt(M(0,i)*M(0,i) + N(0,i)*N(0,i)))
                      - std::atan2(N(0,i), M(0,i));
        // Map: alpha = 0 (horizontal)  -> send 90
        //      alpha < 0 (below)       -> send < 90
        //      alpha > 0 (above)       -> send > 90
        const double logical_deg = 90.0 + (RAD2DEG * alpha[i]);

        // Safety: only clamp AFTER adding the offset, so negative alpha survives
        angles[i] = clampD(logical_deg, 0.0, 180.0);
    }

    // 4) Send once per tick

        std::ostringstream ss;

        for (size_t i=0;i<angles.size();++i) { ss << int(angles[i]); if (i+1<angles.size()) ss << ","; }
        ss << "\n";

        if (sp_ && sp_->isOpen())
        {
            sendAnglesQt(QString::fromStdString(ss.str()));
        }



    // 5) Update knee & GL
    for (size_t j=0; j<6; j++)
    {
        Knee_pos_new(0,j) = servo_arm*cos(alpha[j])*cos(beta[j]) + Servo_pos(0,j);
        Knee_pos_new(1,j) = servo_arm*cos(alpha[j])*sin(beta[j]) + Servo_pos(1,j);
        Knee_pos_new(2,j) = servo_arm*sin(alpha[j]) + Servo_pos(2,j);
    }
    pushGeometryToGL();
}

/**
 * @brief Create and populate the telemetry dock widget.
 *
 * Adds a right-side dock with a grid of live labels (Euler, rates, accelerations)
 * wired via the X-Plane UDP sample handler.
 */

void MainWindow::createTelemetryDock_()
{
    auto *dock = new QDockWidget(tr("Live Telemetry"), this);
    dock->setObjectName("LiveTelemetryDock");
    dock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);

    auto *w = new QWidget(dock);
    auto *grid = new QGridLayout(w);

    int r = 0;
    auto addRow = [&](const QString& name, QLabel*& out)
    {
        auto *label = new QLabel(name, w);
        out = new QLabel("--", w);
        grid->addWidget(label, r, 0);
        grid->addWidget(out,   r, 1);
        ++r;
    };

    addRow("Roll (deg)",  lblRoll_);
    addRow("Pitch (deg)", lblPitch_);
    addRow("Yaw (deg)",   lblYaw_);
    addRow("p (rad/s)",   lblP_);
    addRow("q (rad/s)",   lblQ_);
    addRow("r (rad/s)",   lblR_);
    addRow("Ax (m/s²)",   lblAx_);
    addRow("Ay (m/s²)",   lblAy_);
    addRow("Az (m/s²)",   lblAz_);

    w->setLayout(grid);
    dock->setWidget(w);
    addDockWidget(Qt::RightDockWidgetArea, dock);
}

/**
 * @brief Detect and open a serial port for the Arduino (Qt implementation).
 *
 * Logs available ports, heuristically selects a candidate (ACM/USB and common
 * Arduino USB vendors), configures 115200 8N1, opens the port, updates UI,
 * and wires error handling to auto-recover by closing and starting retries.
 *
 * @note Stops the retry timer on success.
 */

void MainWindow::setupSerialQt()
{
    if (sp_)
    {                           // already have an object
        if (sp_->isOpen()) return;       // already open
        sp_->deleteLater();
        sp_ = nullptr;
    }

    // LOG what Qt sees
    const auto ports = QSerialPortInfo::availablePorts();
    qDebug() << "[Serial] Ports found:" << ports.size();
    for (const QSerialPortInfo& info : ports)
    {
        qDebug() << "  " << info.systemLocation()
        << "| name:" << info.portName()
        << "| desc:" << info.description()
        << "| mfg:"  << info.manufacturer();
    }

    // 1) Try to auto-pick a reasonable port (ttyACM*/ttyUSB*; prefer Arduino-like)
    QString candidate;
    for (const QSerialPortInfo& info : QSerialPortInfo::availablePorts()) {
        const QString name = info.portName();
        const QString sys  = info.systemLocation().toLower(); // e.g. /dev/ttyACM0
        const QString desc = info.description().toLower();
        const QString mfg  = info.manufacturer().toLower();

        // Heuristics: prefer ACM/USB and "arduino"/"silabs"/"wch"/"ftdi"
        const bool looksUsb  = sys.contains("ttyacm") || sys.contains("ttyusb");
        const bool looksArd  = desc.contains("arduino") || mfg.contains("arduino")
                              || mfg.contains("silabs")  || mfg.contains("ftdi") || mfg.contains("wch");
        if (looksUsb && (candidate.isEmpty() || looksArd))
        {
            candidate = name; // keep first, replace with "better" matches
            if (looksArd) break; // strong match found
        }
    }

    if (candidate.isEmpty()) {
        ui->ArduinoStatus->setText("Arduino Status: No serial ports found");
        return;
    }

    // 2) Create and configure the serial port
    sp_ = new QSerialPort(this);
    sp_->setPortName(candidate);
    sp_->setBaudRate(QSerialPort::Baud115200);
    sp_->setDataBits(QSerialPort::Data8);
    sp_->setParity(QSerialPort::NoParity);
    sp_->setStopBits(QSerialPort::OneStop);
    sp_->setFlowControl(QSerialPort::NoFlowControl);

    // 3) Open for write
    if (!sp_->open(QIODevice::ReadWrite))
    {
        ui->ArduinoStatus->setText(QString("Arduino Status: Open failed (%1)")
                                       .arg(sp_->errorString()));

        qWarning() << "[Serial] Open failed for" << candidate << ":" << sp_->errorString();
        sp_->deleteLater(); sp_ = nullptr;
        return;
    }

    serialPortName_ = candidate;
    ui->ArduinoStatus->setText(QString("Arduino Status: Connected (%1)").arg(serialPortName_));
     qDebug() << "[Serial] Connected to" << serialPortName_;
    // 4) Wire error handling – try to recover if the device disappears
    connect(sp_, &QSerialPort::errorOccurred, this, [this](QSerialPort::SerialPortError e){
        if (e == QSerialPort::NoError) return;
        qWarning() << "Serial error:" << sp_->errorString();
        closeSerialQt();
        if (serialRetry_) serialRetry_->start();
    });

    // Stop retrying once connected
    if (serialRetry_) serialRetry_->stop();
}


/**
 * @brief Retry opening the serial port if currently disconnected.
 *
 * Stops retrying when a port becomes open.
 */

void MainWindow::retryOpenSerialQt()
{
    if (sp_ && sp_->isOpen()) { serialRetry_->stop(); return; }
    setupSerialQt();
}
/**
 * @brief Close and dispose of the Qt serial port object; update UI.
 */

void MainWindow::closeSerialQt()
{
    if (!sp_) return;
    if (sp_->isOpen()) sp_->close();
    sp_->deleteLater();
    sp_ = nullptr;
    ui->ArduinoStatus->setText("Arduino Status: Disconnected");
}

/**
 * @brief Send a CSV line of servo angles over serial.
 *
 * Converts the line to UTF-8 and writes to the port; logs on failure and
 * flushes on success.
 *
 * @param line Angle line (e.g., "90,90,90,90,90,90\n").
 */

void MainWindow::sendAnglesQt(const QString& line)
{
    if (!sp_ || !sp_->isOpen()) return;
    const QByteArray bytes = line.toUtf8();
    const qint64 n = sp_->write(bytes);
    if (n == -1) {
        qWarning() << "Serial write failed:" << sp_->errorString();
    } else {
        // Optional: ensure it actually went out; usually not needed
        sp_->flush();        // push to driver
        // sp_->waitForBytesWritten(10); // small blocking wait if you want
    }
}

/**
 * @brief Toggle between Manual and FromSim drive modes.
 *
 * Disables/enables sliders accordingly, clears sim-zero capture flag, and
 * resets platform attitude to zero when returning to manual.
 *
 * @param checked True → FromSim, False → Manual.
 */

void MainWindow::on_chkDriveFromSim_toggled(bool checked)
{
    driveMode_ = checked ? DriveMode::FromSim : DriveMode::Manual;

    //clear the flag
    haveSimZero_ = false;

    // Disable sliders in sim mode so they don’t fight the UDP updates
    ui->Slider_X->setEnabled(!checked);
    ui->Slider_Y->setEnabled(!checked);
    ui->Slider_Z->setEnabled(!checked);
    ui->Slider_PHI->setEnabled(!checked);
    ui->Slider_THETA->setEnabled(!checked);
    ui->Slider_PSI->setEnabled(!checked);

    if (!checked)
    {
        // Back to manual: reset platform target attitude to 0 (visual + pose)
        phi = theta = psi = 0.0;
    }
}


/**
 * @brief Initialize base/top anchor geometry from platform angle parameters.
 *
 * Computes 2D coordinates for platform joints (top) using THETA_R and for
 * base servo centers (bottom) using THETA_P, relative to symmetry axes and
 * global rotation GAMMA. Loads results into Eigen 3×6 matrices with Z=0.
 *
 * @note Establishes Platform_pos_zero and Servo_pos used by IK.
 */


void MainWindow::initGeometryFromAngles_()
{
    // Note: I forgot where I saw this type of a thing being done years ago. Someone shared an example
    // of making things parametric instead of hardcoding geometry (much superior). This is an attempt
    // at trying to utilize that brilliance :D TODO: find out the original source and link


    // Symmetry axes : 30°, -90°, 30° (the last is mirrored in X later)
    constexpr double AXIS1 =  GAMMA;
    constexpr double AXIS2 = -90.0 * DEG2RAD;
    constexpr double AXIS3 =  AXIS1;  // mirror via sign change below

    // --- Platform joints (top plate), using Theta_R ---
    double Pxy[6][2] =
        {
                        { P_RAD * std::cos(AXIS1 + THETA_R),  P_RAD * std::sin(AXIS1 + THETA_R) },
                        { P_RAD * std::cos(AXIS1 - THETA_R),  P_RAD * std::sin(AXIS1 - THETA_R) },
                        { P_RAD * std::cos(AXIS2 + THETA_R),  P_RAD * std::sin(AXIS2 + THETA_R) },
                        { -P_RAD * std::cos(AXIS2 + THETA_R), P_RAD * std::sin(AXIS2 + THETA_R) },
                        { -P_RAD * std::cos(AXIS3 - THETA_R), P_RAD * std::sin(AXIS3 - THETA_R) },
                        { -P_RAD * std::cos(AXIS3 + THETA_R), P_RAD * std::sin(AXIS3 + THETA_R) },
        };

    // --- Base servo centers (bottom plate), using Theta_P ---
    double Bxy[6][2] =
        {
                        { B_RAD * std::cos(AXIS1 + THETA_P),  B_RAD * std::sin(AXIS1 + THETA_P) },
                        { B_RAD * std::cos(AXIS1 - THETA_P),  B_RAD * std::sin(AXIS1 - THETA_P) },
                        { B_RAD * std::cos(AXIS2 + THETA_P),  B_RAD * std::sin(AXIS2 + THETA_P) },
                        { -B_RAD * std::cos(AXIS2 + THETA_P), B_RAD * std::sin(AXIS2 + THETA_P) },
                        { -B_RAD * std::cos(AXIS3 - THETA_P), B_RAD * std::sin(AXIS3 - THETA_P) },
                        { -B_RAD * std::cos(AXIS3 + THETA_P), B_RAD * std::sin(AXIS3 + THETA_P) },
        };



    // Load into Eigen 3x6 members (Z=0 in their own planes)
    for (int i = 0; i < 6; ++i)
    {
        Platform_pos_zero(0,i) = Pxy[i][0];
        Platform_pos_zero(1,i) = Pxy[i][1];
        Platform_pos_zero(2,i) = 0.0;

        Servo_pos(0,i) = Bxy[i][0];
        Servo_pos(1,i) = Bxy[i][1];
        Servo_pos(2,i) = 0.0;
    }
}

