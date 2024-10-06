#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSlider>
// For serial com
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <cstring> // for memset
#include <Eigen/Dense>


namespace Ui {
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

    //for serial
    void setupSerial();//(int &fd, struct termios &settings);
    void sendToSerial(int fd, const char* data);
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
    // Initialize serial com with Arduino
    int fd; // file descriptor
    struct termios settings;  // termios config for the serial port

    // Platform geometry
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



    QSlider *Slider_X; // declare Slider_X as a member of the class
    QSlider *Slider_Y;
    QSlider *Slider_Z;
    QSlider *Slider_PHI;
    QSlider *Slider_THETA;
    QSlider *Slider_PSI;



};

#endif // MAINWINDOW_H
