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
        servo_arm(18),
        servo_leg(120),
        beta{0, -M_PI / 3, -2 * M_PI / 3, -M_PI, -4 * M_PI / 3, -5 * M_PI / 3},        //beta{M_PI / 2, M_PI / 6, -M_PI/6,  -M_PI / 2, -5 * M_PI / 6, -2* M_PI /3},//beta{M_PI / 3, -2 * M_PI / 3, M_PI, 0, 5 * M_PI / 3, 2 * M_PI / 3},
        height(115) // Initialization of height

{

    // Make sure the sliders are all connected
    ui->setupUi(this);
    connect(ui->Slider_X, SIGNAL(valueChanged(int)), this, SLOT(on_Slider_X_valueChanged(int)));
    connect(ui->Slider_Y, SIGNAL(valueChanged(int)), this, SLOT(on_Slider_Y_valueChanged(int)));
    connect(ui->Slider_Z, SIGNAL(valueChanged(int)), this, SLOT(on_Slider_Z_valueChanged(int)));
    connect(ui->Slider_PHI, SIGNAL(valueChanged(int)), this, SLOT(on_Slider_PHI_valueChanged(int)));
    connect(ui->Slider_THETA, SIGNAL(valueChanged(int)), this, SLOT(on_Slider_THETA_valueChanged(int)));
    connect(ui->Slider_PSI, SIGNAL(valueChanged(int)), this, SLOT(on_Slider_PSI_valueChanged(int)));



    //======================Platform Geometry ====================\\
    // Initialise platform related constants here

    // Platform points (top) Pi
    Platform_pos_zero << -41.4, -53.9, -12.5, 12.5, 53.9, 41.4, // First row
                         38.3, 16.7, -55, -55, 16.7, 38.3,      // Second row
                         0, 0, 0, 0, 0, 0;                     // Third row



    // Servo points (base) //! z changed to all zeros // Trying to make it sy
    Servo_pos << -46.5, -90.5, -43.5, 43.5, 90.5, 46.5,  // First row
                  74, 1.1, -81, -81, 1.14, 74,           // Second row
                  0, 0, 0, 0, 0, 0;                     // Third row


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

MainWindow::~MainWindow()
{

    delete ui;


}

void MainWindow::on_homeButton_clicked()
{
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
       alpha_home_deg = rad2deg*alpha_home;


       // Let's try to workout the servo arm/leg join positions


       // Changed knee position


       for (size_t j=0; j<6; j++)
                   {
                       Knee_pos_home(0,j) = servo_arm*cos(alpha_home)*cos(beta[j]) + Servo_pos(0,j);
                       Knee_pos_home(1,j) = servo_arm*cos(alpha_home)*sin(beta[j]) + Servo_pos(1,j);
                       Knee_pos_home(2,j) = servo_arm*sin(alpha_home) + Servo_pos(2,j);
                   }











}
void MainWindow::setupSerial()//(int &fd, struct termios &settings)
{
    // This just changes the text in the GUI
     ui->ArduinoStatus->setText("Arduino Status: Checking connection...");
    // This opens a serial com and sets a file descriptor
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

        // When we arrive here, we know we are connected so we alter the GUI text
       ui->ArduinoStatus->setText("Arduino Status: Connected");

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
    // Move this later
    setupSerial();




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

                alpha(0,i) = asin(L(0,i) / sqrt(std::pow(M(0,i), 2) + std::pow(N(0,i), 2))) - atan(N(0,i) / M(0,i));
                servo_deg(0,i) = rad2deg*(alpha[i]);
                angles[i] = clamp(servo_deg[i],0.0,180.0); // getting the angles ready in the angles array


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
            sendToSerial(fd, angleStream.str().c_str());



            ::close(fd); // Close the serial port after use



         //Changed knee position}


                    for (size_t j=0; j<6; j++)
                    {
                        Knee_pos_new(0,j) = servo_arm*cos(alpha[j])*cos(beta[j]) + Servo_pos(0,j);
                        Knee_pos_new(1,j) = servo_arm*cos(alpha[j])*sin(beta[j]) + Servo_pos(1,j);
                        Knee_pos_new(2,j) = servo_arm*sin(alpha[j]) + Servo_pos(2,j);
                    }








}

void MainWindow::on_abortButton_clicked()
{
    QApplication::quit();
}


void MainWindow::on_Slider_X_valueChanged(int value)
{
    double X =  1.0*value;//(ui->Slider_X->value());
    setX(X);
    t_input << X, Y, Z;
    T = t_home + t_input;
    on_startButton_clicked();


}

void MainWindow::on_Slider_Y_valueChanged(int value)
{
    double Y = 1.0*value;//(ui ->Slider_Y ->value());
    setY(Y);
    t_input << X, Y, Z;
    T = t_home + t_input;
    on_startButton_clicked();
}

void MainWindow::on_Slider_Z_valueChanged(int value)
{
    double Z = 1.0*value;//(ui ->Slider_Z -> value());
    setZ(Z);
    t_input << X, Y, Z;
    T = t_home + t_input;
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
