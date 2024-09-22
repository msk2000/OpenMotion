#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <iostream>
#include <Eigen/Dense>
#include <math.h>
#include <array>
#include <QDebug>
// For serial com
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <cstring> // for memset


#define rad2deg 180 / M_PI
#define deg2rad M_PI / 180


using Eigen::MatrixXd;
using Eigen::VectorXd;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)

{


    ui->setupUi(this);
    connect(ui->Slider_X, SIGNAL(valueChanged(int)), this, SLOT(on_Slider_X_valueChanged(int)));
    connect(ui->Slider_Y, SIGNAL(valueChanged(int)), this, SLOT(on_Slider_Y_valueChanged(int)));
    connect(ui->Slider_Z, SIGNAL(valueChanged(int)), this, SLOT(on_Slider_Z_valueChanged(int)));
    connect(ui->Slider_PHI, SIGNAL(valueChanged(int)), this, SLOT(on_Slider_PHI_valueChanged(int)));
    connect(ui->Slider_THETA, SIGNAL(valueChanged(int)), this, SLOT(on_Slider_THETA_valueChanged(int)));
    connect(ui->Slider_PSI, SIGNAL(valueChanged(int)), this, SLOT(on_Slider_PSI_valueChanged(int)));

}

MainWindow::~MainWindow()
{

    delete ui;


}

void MainWindow::on_homeButton_clicked()
{

       ui ->Slider_X->setValue(0);
       ui ->Slider_Y->setValue(0);
       ui ->Slider_Z->setValue(0);

       ui ->Slider_PHI->setValue(0);
       ui ->Slider_THETA->setValue(0);
       ui ->Slider_PSI->setValue(0);




    //All the constant stuff
        double X, Y, Z, phi, theta, psi;
        double servo_arm = 18;
        double servo_leg = 140;
        double beta[6] =
        {
                            M_PI / 3,
                            -2 * M_PI / 3,
                            M_PI,
                            0,
                            5 * M_PI / 3,
                            2 * M_PI / 3
        }; //! Changes home calcs
        double height = 155;
        int FLAG = 0;

        // Platform points (top) Pi
        MatrixXd Platform_pos_zero{
            {-41.4, 38.3, 0},
            {-53.9, 16.7, 0},
            {-12.5, -55, 0},
            {12.5, -55, 0},
            {53.9, 16.7, 0},
            {41.4, 38.3, 0}};
        Platform_pos_zero.transposeInPlace();


        // Servo points (base) //! z changed to all zeros // Trying to make it sy
        MatrixXd Servo_pos{
            {-46.5, 74, 0},
            {-90.5, 1.1, 0},
            {-43.5, -81, 0},
            {43.5, -81, 0},
            {90.5, 1.14, 0},
            {46.5, 74, 0}};
        Servo_pos.transposeInPlace();

        // Rotation matrix (R = Rz*Ry*Rx) used to take platform stuff to base frame
        MatrixXd R_PB{
            {cos(psi) * cos(theta), (-sin(psi) * cos(phi)) + (cos(psi) * sin(theta) * sin(phi)), (sin(psi) * sin(phi)) + (cos(psi) * sin(theta) * cos(phi))},
            {sin(psi) * cos(theta), (cos(psi) * cos(phi)) + (sin(psi) * sin(theta) * sin(phi)), (-cos(psi) * sin(phi)) + (sin(psi) * sin(theta) * cos(phi))},
            {-sin(theta), cos(theta) * sin(phi), cos(theta) * cos(phi)}

        };




    // Initialising to home position
        while(FLAG == 0)
        {
           //Parameters (3 trans inputs and 3 rot inputs)
           X = Y = Z = phi = theta = psi = 0;

           // Height of the platform when servo arm is perpendicular to leg
           double h_0 = sqrt(std::pow(servo_leg,2) + std::pow(servo_arm,2)
                   - std::pow(Platform_pos_zero(0,0) - Servo_pos(0,0),2)
                   - std::pow(Platform_pos_zero(1,0) - Servo_pos(1,0),2))
                   - Platform_pos_zero(2,0);


           // Platform points when zero rotation and translation
           Eigen::Matrix<double, 3, 1> t_home;
           t_home << 0, 0, h_0;
           Eigen::Matrix<double, 3, 1> t_input;
           t_input << X, Y, Z;
           Eigen::Matrix<double, 3, 1> T;
           T = t_home + t_input;


           // Calculate platform's home position (New_pos)
           MatrixXd Rotated_platform = R_PB * Platform_pos_zero;
           //MatrixXd New_pos = (T.array().replicate<1, 6>().matrix()).array() + Rotated_platform.array();


           MatrixXd New_pos = T.replicate<1, 6>().array() + Rotated_platform.array(); // q


           // Calculate angle of the servo arm at home position
           // First find the linear Leg length

           MatrixXd lin_leg_lengths = New_pos - Servo_pos;

           // The .colwise().norm() method calculates the Euclidean norm of each column,
           // which corresponds to the length of each leg vector
           MatrixXd virtual_leg_lengths = (lin_leg_lengths).colwise().norm();




           // Due to platform symmetry, we can only consider the leg with 0 beta //!Are we symmetric?
           double L_home = 2 * std::pow(servo_arm, 2);
           double M_home = 2 * servo_arm * (New_pos(0,0) - Servo_pos(0,0));
           double N_home = 2 * servo_arm * (h_0 + New_pos(2,3));

           double alpha_home = asin(L_home/sqrt(pow(M_home,2)+pow(N_home,2))) - atan(M_home/N_home);
           double alpha_home_deg = rad2deg*alpha_home;


          // Let's try to workout the servo arm/leg join positions


        //Changed knee position
        Eigen::Matrix<double,3,6> Knee_pos_home;

                   for (size_t j=0; j<6; j++)
                   {
                       Knee_pos_home(0,j) = servo_arm*cos(alpha_home)*cos(beta[j]) + Servo_pos(0,j);
                       Knee_pos_home(1,j) = servo_arm*cos(alpha_home)*sin(beta[j]) + Servo_pos(1,j);
                       Knee_pos_home(2,j) = servo_arm*sin(alpha_home) + Servo_pos(2,j);
                   }









            FLAG = 1;
        };
}
void MainWindow::setupSerial(int &fd, struct termios &settings)
{
    fd = open("/dev/ttyACM0", O_WRONLY | O_NDELAY | O_NOCTTY);
        if (fd < 0)
        {
            perror("Error opening serial port");
            exit(-1);
        }

        memset(&settings, 0, sizeof(settings)); // Clear the struct
        settings.c_cflag = B9600 | CS8 | CLOCAL | CREAD; // Set baudrate, data size, ignore modem control
        settings.c_iflag = IGNPAR; // Ignore parity errors
        settings.c_oflag = 0; // No special output processing
        settings.c_lflag = 0; // No special local modes

        // Apply the settings
        tcflush(fd, TCIFLUSH); // Clear input buffer
        tcflush(fd, TCOFLUSH); // Clear output buffer
        tcsetattr(fd, TCSANOW, &settings); // Apply the settings immediately
}

void MainWindow::sendToSerial(int fd, const char* data)
{
    int len = strlen(data);  // Length of the string to send
    int bytesWritten = write(fd, data, len);  // Write to the serial port
    if (bytesWritten < 0) {
        perror("Error writing to serial port");
        return;
    }
    qDebug() << "Sent to serial:" << data;
}

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

void MainWindow::on_startButton_clicked()
{
    // Initialize serial com with Arduino
    int fd; // file descriptor
    struct termios settings; // struct for storing termios config
    int serialPosition; // servo position

    // Call setupSerial to initialize the serial port
     setupSerial(fd, settings);

    //Control a single servo for testing
    double angle = ui->Slider_X->value();  // Get the angle from the slider (or any input control)
    angle = clamp(angle, 0.0, 180.0);  //    Ensure the angle is between 0 and 180 degrees

    // Convert angle to string for sending over serial
    std::string angleStr = std::to_string(static_cast<int>(angle)) + "\n";

    // Send angle to Arduino using custom serial method
    sendToSerial(fd, angleStr.c_str()); // Replace with your own function

    // Print the current angle to the debug window
    qDebug() << "Servo angle sent:" << angle;

    ::close(fd); // Close the serial port after use




    /////////////////////////////////////
    /// Setting defautls








    double servo_arm = 18;
    double servo_leg = 140;
    double beta[6] = {
                        M_PI / 3,
                        -2 * M_PI / 3,
                        M_PI,
                        0,
                        5 * M_PI / 3,
                        2 * M_PI / 3}; //! Changes home calcs
    double height = 155;


    // Platform points (top) Pi
    MatrixXd Platform_pos_zero{
        {-41.4, 38.3, 0},
        {-53.9, 16.7, 0},
        {-12.5, -55, 0},
        {12.5, -55, 0},
        {53.9, 16.7, 0},
        {41.4, 38.3, 0}};
    Platform_pos_zero.transposeInPlace();
   // (Platform_pos_zero); // debug

    // Servo points (base) //! z changed to all zeros // Trying to make it sy
    MatrixXd Servo_pos{
        {-46.5, 74, 0},
        {-90.5, 1.1, 0},
        {-43.5, -81, 0},
        {43.5, -81, 0},
        {90.5, 1.14, 0},
        {46.5, 74, 0}};
    Servo_pos.transposeInPlace();
   // (Servo_pos); // debug

    // Rotation matrix (R = Rz*Ry*Rx) used to take platform stuff to base frame
    MatrixXd R_PB{
        {cos(psi) * cos(theta), (-sin(psi) * cos(phi)) + (cos(psi) * sin(theta) * sin(phi)), (sin(psi) * sin(phi)) + (cos(psi) * sin(theta) * cos(phi))},
        {sin(psi) * cos(theta), (cos(psi) * cos(phi)) + (sin(psi) * sin(theta) * sin(phi)), (-cos(psi) * sin(phi)) + (sin(psi) * sin(theta) * cos(phi))},
        {-sin(theta), cos(theta) * sin(phi), cos(theta) * cos(phi)}

    };
    double FLAG = 1;

    // When platform is moving
        while(FLAG == 1)
        {



        // Height of the platform when servo arm is perpendicular to leg
           double h_0 = sqrt(std::pow(servo_leg,2) + std::pow(servo_arm,2)
                   - std::pow(Platform_pos_zero(0,0) - Servo_pos(0,0),2)
                   - std::pow(Platform_pos_zero(1,0) - Servo_pos(1,0),2))
                   - Platform_pos_zero(2,0);


           // Platform points when zero rotation and translation
           Eigen::Matrix<double, 3, 1> t_home;
           t_home << 0, 0, h_0;
           Eigen::Matrix<double, 3, 1> t_input;
           t_input << X, Y, Z;
           qDebug() << "X here is :" << X;
           Eigen::Matrix<double, 3, 1> T;
           T = t_home + t_input;

           //! testing rot import
           // Rotation matrix (R = Rz*Ry*Rx) used to take platform stuff to base frame
        MatrixXd R_PB{
            {cos(psi) * cos(theta), (-sin(psi) * cos(phi)) + (cos(psi) * sin(theta) * sin(phi)), (sin(psi) * sin(phi)) + (cos(psi) * sin(theta) * cos(phi))},
            {sin(psi) * cos(theta), (cos(psi) * cos(phi)) + (sin(psi) * sin(theta) * sin(phi)), (-cos(psi) * sin(phi)) + (sin(psi) * sin(theta) * cos(phi))},
            {-sin(theta), cos(theta) * sin(phi), cos(theta) * cos(phi)}

        };

           // Calculate platform's transformed position(New_pos)
           MatrixXd Rotated_platform = R_PB * Platform_pos_zero;
           //MatrixXd New_pos = (T.array().replicate<1, 6>().matrix()).array() + Rotated_platform.array();


           MatrixXd New_pos = T.replicate<1, 6>().array() + Rotated_platform.array(); // q


           // Calculate angle of the servo arm at NEW position
           // First find the linear Leg length

           MatrixXd lin_leg_lengths = New_pos - Servo_pos;

           // The .colwise().norm() method calculates the Euclidean norm of each column,
           // which corresponds to the length of each leg vector
           MatrixXd virtual_leg_lengths = (lin_leg_lengths).colwise().norm();


           // Calculate the servo angles for each leg

           Eigen::Matrix<double,1,6> L;
            for (size_t i=0;i<6;i++)
            {

                L(0,i) = std::pow(virtual_leg_lengths(0,i),2)
                         - ((std::pow(servo_leg,2)) - (std::pow(servo_arm,2)));

            }



            Eigen::Matrix<double,1,6> M;
            for (size_t i=0;i<6;i++)
            {
                M(0,i) = 2*servo_arm*(New_pos(2,i)-Servo_pos(2,i));

            }



            Eigen::Matrix<double,1,6> N;
            for (size_t i=0;i<6;i++)
            {
                double x_diff  = New_pos(0,i) - Servo_pos(0,i); // intermediate calc
                double y_diff  = New_pos(1,i) - Servo_pos(1,i);
                N(0,i) = 2*servo_arm*((cos(beta[i])*x_diff)+(sin(beta[i])*y_diff));
            }



            // Now we can calculate the servo angles
            Eigen::Matrix<double,1,6>alpha;
            Eigen::Matrix<double,1,6>servo_deg;

            for (size_t i = 0; i <6; i++)
            {

                alpha(0,i) = asin(L(0,i) / sqrt(std::pow(M(0,i), 2) + std::pow(N(0,i), 2))) - atan(N(0,i) / M(0,i));
                servo_deg(0,i) = rad2deg*(alpha[i]);


            }


         //Changed knee position
         Eigen::Matrix<double,3,6> Knee_pos_new;

                    for (size_t j=0; j<6; j++)
                    {
                        Knee_pos_new(0,j) = servo_arm*cos(alpha[j])*cos(beta[j]) + Servo_pos(0,j);
                        Knee_pos_new(1,j) = servo_arm*cos(alpha[j])*sin(beta[j]) + Servo_pos(1,j);
                        Knee_pos_new(2,j) = servo_arm*sin(alpha[j]) + Servo_pos(2,j);
                    }



            FLAG = 0;



        };




}

void MainWindow::on_abortButton_clicked()
{
    QApplication::quit();
}


void MainWindow::on_Slider_X_valueChanged(int value)
{
    double X =  1.0*value;//(ui->Slider_X->value());
    qDebug() << "X value changed to" << X;
    setX(X);
    on_startButton_clicked();


}

void MainWindow::on_Slider_Y_valueChanged(int value)
{
    double Y = 1.0*value;//(ui ->Slider_Y ->value());
    setY(Y);
    on_startButton_clicked();
}

void MainWindow::on_Slider_Z_valueChanged(int value)
{
    double Z = 1.0*value;//(ui ->Slider_Z -> value());
    setZ(Z);
    on_startButton_clicked();
}

void MainWindow::on_Slider_PHI_valueChanged(int value)
{
    double phi = deg2rad*value;//(ui->Slider_PHI ->value()) ;
    setPHI(phi);
    on_startButton_clicked();
}

void MainWindow::on_Slider_THETA_valueChanged(int value)
{
    double theta = deg2rad*value;//(ui ->Slider_THETA ->value());
    setTHETA(theta);
    on_startButton_clicked();
}

void MainWindow::on_Slider_PSI_valueChanged(int value)
{
    double psi = deg2rad*value;//(ui ->Slider_PSI -> value());
    setPSI(psi);
    on_startButton_clicked();
}

void MainWindow::setX(double value)
{
    X = value;
}
void MainWindow::setY(double value)
{
    Y = value;
}

void MainWindow::setZ(double value)
{
    Z = value;
}
void MainWindow::setPHI(double value)
{
    phi = value;
}
void MainWindow::setTHETA(double value)
{
    theta = value;
}
void MainWindow::setPSI(double value)
{
    psi = value;
}
