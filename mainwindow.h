#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSlider>
// For serial com
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <cstring> // for memset


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

    QSlider *Slider_X; // declare Slider_X as a member of the class
    QSlider *Slider_Y;
    QSlider *Slider_Z;
    QSlider *Slider_PHI;
    QSlider *Slider_THETA;
    QSlider *Slider_PSI;



};

#endif // MAINWINDOW_H
