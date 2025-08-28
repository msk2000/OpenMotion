/******************************************************************************
 * @file    mainwindow.h
 * @brief   Declaration of MainWindow — primary controller for the Open Motion
 *          6-DOF platform application (UI, IK, telemetry, serial, rendering).
 *
 * Responsibilities:
 *   - Wire UI controls and slider callbacks (manual drive mode).
 *   - Handle sim drive via X-Plane UDP (XPlaneUdpReceiver).
 *   - Maintain platform state (X,Y,Z, φ,θ,ψ) and geometry (Eigen 3×6).
 *   - Provide IK intermediates and computed servo angles for 6 servos.
 *   - Manage Arduino serial I/O (QtSerialPort) with auto-detect/retry.
 *   - Bridge geometry to ImGuiGLWidget for 3D visualization.
 *   - Expose a live telemetry dock (Euler, body rates, accelerations).
 *
 * Conventions:
 *   - Geometry uses millimeters and radians unless noted; slider angles are
 *     in degrees (converted in the implementation).
 *   - Eigen matrices are column-major 3×6 sets: rows = {x,y,z}, cols = joints.
 *
 * Part of the Open Motion project (6DOF Motion Simulator).
 ******************************************************************************/

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSlider>
#include <cstring> // for memset
#include <Eigen/Dense>
#include "xplaneudpreceiver.h"
#include <QTimer>
#include <QLabel>
#include <QDockWidget>
#include <QSerialPort>
#include <QSerialPortInfo>

// forward declare
class ImGuiGLWidget;



namespace Ui
{
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    void setX(double value);
    void setY(double value);
    void setZ(double value);
    void setPHI(double value);
    void setTHETA(double value);
    void setPSI(double value);

private slots:
    void on_startButton_clicked();

    void on_abortButton_clicked();

    void on_homeButton_clicked();

    void on_Slider_X_valueChanged(int value);

    void on_Slider_Y_valueChanged(int value);

    void on_Slider_Z_valueChanged(int value);

    void on_Slider_PHI_valueChanged(int value);

    void on_Slider_THETA_valueChanged(int value);

    void on_Slider_PSI_valueChanged(int value);

    void on_chkDriveFromSim_toggled(bool checked);


    double clamp(double value, double min, double max);





private:
    Ui::MainWindow *ui;
    double X;//=  1.0*(ui->Slider_X->value());
    double Y;//= 1.0*(ui ->Slider_Y ->value());
    double Z;//= 1.0*(ui ->Slider_Z -> value());
    double phi;//= deg2rad*(ui->Slider_PHI ->value()) ;//-0.087266462599717;
    double theta;//= deg2rad*(ui ->Slider_THETA ->value());//0.226892802759263;
    double psi;// = deg2rad*(ui ->Slider_PSI -> value());//0.2;//0.191986217719376;
    std::array<double, 6> angles;
    double servo_arm;
    double servo_leg;
    double beta[6];
    double height;


    //============== Platform geometry ===================
    Eigen::Matrix<double, 3, 6> Platform_pos_zero;
    Eigen::Matrix<double, 3, 6> Servo_pos;
    Eigen::Matrix<double,3,3> R_PB; //rotation matrix
    Eigen::Matrix<double, 3, 1> t_home;
    Eigen::Matrix<double, 3, 1> t_input;
    Eigen::Matrix<double, 3, 1> T;
    double h_0;
    Eigen::Matrix<double, 3, 6> Rotated_platform;
    Eigen::Matrix<double, 3, 6> New_pos;
    Eigen::Matrix<double, 3, 6> lin_leg_lengths;
    Eigen::Matrix<double, 1, 6> virtual_leg_lengths;
    double L_home;
    double M_home;
    double N_home;
    Eigen::Matrix<double,1,6> L;
    Eigen::Matrix<double,1,6> M;
    double x_diff; // intermediate calc
    double y_diff;
    Eigen::Matrix<double,1,6> N;
    Eigen::Matrix<double,1,6>alpha;
    Eigen::Matrix<double,1,6>servo_deg;
    Eigen::Matrix<double,3,6> Knee_pos_new;
    double alpha_home;
    double alpha_home_deg;
    Eigen::Matrix<double,3,6> Knee_pos_home;

    void initGeometryFromAngles_();

    //=========== GUI ===============================
    QSlider *Slider_X; // declare Slider_X as a member of the class
    QSlider *Slider_Y;
    QSlider *Slider_Z;
    QSlider *Slider_PHI;
    QSlider *Slider_THETA;
    QSlider *Slider_PSI;

    void pushGeometryToGL();

    ImGuiGLWidget* glw_ = nullptr;

    XPlaneUdpReceiver* udpRx_ = nullptr; // from the previous fix
    QTimer* tick_ = nullptr;             // for periodic updates
    void stepPlatformAndSerial();

    //=================== Telemetry labels ==============
    QLabel *lblRoll_ = nullptr, *lblPitch_ = nullptr, *lblYaw_ = nullptr;
    QLabel *lblP_ = nullptr, *lblQ_ = nullptr, *lblR_ = nullptr;
    QLabel *lblAx_ = nullptr, *lblAy_ = nullptr, *lblAz_ = nullptr;

    void createTelemetryDock_();

    // ============== Qt SerialPort ===============
    void setupSerialQt();                 // open (auto-detect) & configure
    void closeSerialQt();                 // close safely
    void sendAnglesQt(const QString&);    // write line to serial
    void retryOpenSerialQt();             // periodic retry if unplugged

    QSerialPort* sp_ = nullptr;
    QTimer* serialRetry_ = nullptr;       // retry timer if device missing
    QString serialPortName_;              // for UI/status

    // ============= Mode Settings ==================
    // Drive mode
    enum class DriveMode { Manual, FromSim };
    DriveMode driveMode_ = DriveMode::Manual;

    // Sim-origin capture (deg) and flag
    bool haveSimZero_ = false;
    double roll0_deg = 0.0, pitch0_deg = 0.0, yaw0_deg = 0.0;

    // Last sample + timestamp for watchdog
    bool lastHaveSample_ = false;
    MotionSample lastSample_{};
    qint64 lastSimMs_ = 0;

    // Simple gains/limits for platform angles [NEEDS TUNING and actually using brains]
    struct
    {
        // attitude
        double kRoll   = 1.0;   // deg->deg gain (used against sim deltas)
        double kPitch  = 1.0;
        double kYaw    = 1.0;
        double maxRoll = 15.0 * M_PI/180;
        double maxPitch= 15.0 * M_PI/180;
        double maxYaw  = 15.0 * M_PI/180;

        // NEW: translations (mm-per-slider-tick or mm-per-sim-unit)
        double kX = 2.0;   // try 2–5 mm per tick to start
        double kY = 2.0;
        double kZ = 2.0;

        // Safety clamps (mm)
        double maxX = 80.0;
        double maxY = 80.0;
        double maxZ = 90.0;   // be mindful of horn limits below-horizontal
    } cue_;
    bool viewFlipY_ = true;  // flip Y in 3D view to fix roll sense

    struct AxisSign
    {
        double roll  = -1.0; // <- invert roll (fixes for “bank left/right”)
        double pitch = +1.0; // adjust if you find pitch is reversed
        double yaw   = +1.0; // adjust if yaw is reversed
    } axis_;


};

#endif // MAINWINDOW_H
