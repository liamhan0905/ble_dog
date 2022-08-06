// In linux place eigen folder in /usr/local/include
// compile with -- g++ -I /usr/include/eigen3 <cpp file name> -o <name of compiled file>
// To compile multiple files, use this structure (for linux):
// compiling: g++ -I /usr/include/eigen3 -c <cpp file name1> <cpp file name2> etc.
// linking:   g++ -I /usr/include/eigen3 <o file name1> <o file name2> etc. -o <executable name>
// link for embedded programming with serial ports:
// blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp

// Look to here: https://www.programmingelectronics.com/serial-read/

#include "pathing/dogPathing.h"

#include <stdio.h>
#include <string.h>

#include <arpa/inet.h>

// Linux Headers -- this code is meant to be run on unix / linux systems
//#include <fcntl.h> // Contains file controls like 0_RDWR
//#include <errno.h> // Error integer and strerror() function
//#include <termios.h> // Contains POSIX terminal control definitions
//#include <unistd.h> // write(), read(), close()

#include "SerialPort.hpp"



#include <iostream>
#include <fstream>
#include <math.h>  /* atan2 */


#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <vector> // For storing multiple end effector configurations into an iterable vector

using namespace Eigen;
using Eigen::Matrix;


using namespace std;

using namespace mn::CppLinuxSerial;


class SpotHandler
{
    public:
        SpotHandler();                  // Constructor
        void sendTrajectory(string file, vector<Matrix<float, 3, 1> > trajectory);
        //void writeToFile(string file, Matrix<float, 3, 1> data);
        void writeToFile(string file, float data[]);
        void readFromFile(string file);
        vector<Matrix<float, 3, 1> > createJointTrajectory(Matrix<float, 3, 1> theta_list_start, Matrix<float, 3, 1> theta_list_end, float total_motion_time, int number_descrete_points, int time_scaling_method);
        // Possibly make function that takes in a joint trajectory of a dynamic by 3 size, then
        //   returns that trajectory as a vector of 3x1 matrices
    private:
        DogPathing myPath;


};

SpotHandler::SpotHandler()              // Constructor
{

}


void SpotHandler::sendTrajectory(string file, vector<Matrix<float, 3, 1> > trajectory)
{
    // Precondition: a string representing the name of the target device
    //               a vector of 3x1 matrices that represents the desired trajectory
    // Postcondition: Binary data is sent to the target file repeatedly, once for each matrix in
    //                  the given trajectory
    for (Matrix<float, 3, 1> jointSet : trajectory)
    {
        //cout << "data prep: " << jointSet[0] << endl;
        //cout << "data prep: " << jointSet[1] << endl;
        //cout << "data prep: " << jointSet[2] << endl;

        // Create an array of floats to send using elements of jointSet
        float floatSet[jointSet.size()];
        floatSet[0] = jointSet[0];
        floatSet[1] = jointSet[1];
        floatSet[2] = jointSet[2];

        cout << "testing: ";
        for (float val : floatSet)
        {
            cout << val << " ";
        }
        cout << endl;

        //writeToFile(file, jointSet);
        writeToFile(file, floatSet);
        //readFromFile(file);
    }
}






// old parameters: string file, Matrix<float, 3, 1> data
void SpotHandler::writeToFile(string file, float data[])
{

    /*
    // Doing stuff to make sending data work...
    int serial_port = open(file.c_str(), O_RDWR);
    if (serial_port < 0)
    {
        printf("Error %i from open: %s\n", errno, strerror(errno));
    }
    // Creating termios struct
    struct termios tty;
    // Read in existing settings, and handle any error
    if (tcgetattr(serial_port, &tty) != 0)
    {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    }


    // *** Control Modes (c_cflags) ***
    tty.c_cflag &= ~PARENB;                         // Clear parity bit, disabling parity
                                                    // &= is bitwise and
    tty.c_cflag &= ~CSTOPB;                         // Clear stop field, only one stop bit is used in communication

    tty.c_cflag &= ~CSIZE;                          // Clear all the size bits
    tty.c_cflag |= CS8;                             // Setting size bits to 8 bits per byte

    tty.c_cflag &= ~CRTSCTS;                        // Disable RTS / CTS hardware flow control
    tty.c_cflag |= CREAD | CLOCAL;                  // Turn on READ and ignore ctrl lines (CLOCAL = 1)
                                                    // This lets us read data   
    // *** Control Modes (c_cflags)***

    // *** Local Modes (c_lflag) *** 
    tty.c_lflag &= ~ICANON;                         // Disable Canonical mode

    tty.c_lflag &= ~ECHO;                           // Disable echo
    tty.c_lflag &= ~ECHOE;                          // Disable erasure
    tty.c_lflag &= ~ECHONL;                         // Disable new-line echo
    tty.c_lflag &= ~ISIG;                           // Disable interpretation of INTR, QUIT, and SUSP

    // *** Local Modes (c_lflag) *** 

    */

    //int binarySize = 12; incoming data here has a size of 8 bytes?
    //int binarySize = sizeof(&data);                              // 4 is the amount of bytes in a float
    int binarySize = 12;
    float* dataPtr = NULL;                                         // 
    dataPtr = data;                                                // dataPtr gets data, which is a pointer
                                                                   //   to the data float array
    /*
    uint32_t* swapBytes = (uint32_t*)dataPtr;

    for (int i = 0; i < 4; i++)
    {
        swapBytes[i] = htonl(swapBytes[i]);
    }
    */

    uint8_t* binaryData = (uint8_t*)dataPtr;


    vector<uint8_t> binaryDataVector;

    for (int i = 0; i < binarySize; i++)
    {
        cout << "binaryData at " << i << ": " << binaryData[i] << endl;
        binaryDataVector.push_back(binaryData[i]);
    }



    // Creating serial port object using SerialPort library
    SerialPort serialPort(file, BaudRate::B_57600, NumDataBits::EIGHT, Parity::NONE, NumStopBits::ONE);
    serialPort.SetTimeout(-1);
    serialPort.Open();

    // Write to file
    serialPort.WriteBinary(binaryDataVector);


    serialPort.Close();

    // Trying to write to a regular file with the serial port library
    SerialPort testerPort("testerData.dat", BaudRate::B_57600, NumDataBits::EIGHT, Parity::NONE, NumStopBits::ONE);
    testerPort.SetTimeout(-1);
    testerPort.Open();
    testerPort.WriteBinary(binaryDataVector);
    testerPort.Close();




    /*

    //int binarySize = 12;
    int binarySize = 4*sizeof(&data);                              // 4 is the amount of bytes in a float

    // Make a matrix pointer, have it get the address of data
    //Matrix<float, 3, 1>* dataPtr;
    float* dataPtr;
    //dataPtr = & data;
    dataPtr = data;

    // Create ostream object instance to process file writing
    ofstream arduinoStream;

    // Declaring and initilizing binary data that's to be sent to the arduino
    // size should eventually be set to be based on the number of elements in data
    // binaryData allocates 12 bytes b/c each floating point number takes 4 bytes of memory
    char* binaryData = new char[binarySize];
    binaryData = (char*)dataPtr;                // binaryData gets dataPtr but cast as a character pointer


    
    // Opening file
    arduinoStream.open(file, ios::out | ios::binary);
    if (arduinoStream.is_open())                        // If file is open
    {
        arduinoStream.write(binaryData, binarySize);
        arduinoStream.close();
    }
    */
}









// Reads data from the given file -- should probably include a position at some point
void SpotHandler::readFromFile(string file)
{
    int binarySize = 12;


    // Create ifstream object to read data from file
    ifstream arduinoStream;

    // Declaring char array to add data to
    char dataFromFile[binarySize];

    // Make a float pointer to the data array dataFromFile
    float* dataPtr;

    // Open arduinoStream
    arduinoStream.open(file, ios::in | ios::binary);
    if (arduinoStream.is_open())
    {
        arduinoStream.read(dataFromFile, binarySize);
        for (char data : dataFromFile)
        {
            cout << "Data in File: " << data << endl;
        }

        //dataPtr = (Matrix<float, 3, 1>*) dataFromFile;     for reading data as a matrix
        dataPtr = (float*) dataFromFile;

        arduinoStream.close();
    }

    /*
    // Creating serial port object using SerialPort library
    SerialPort serialPort(file, BaudRate::B_57600, NumDataBits::EIGHT, Parity::NONE, NumStopBits::ONE);
    serialPort.SetTimeout(-1);
    serialPort.Open();

    // Write to file
    serialPort.WriteBinary(binaryDataVector);


    serialPort.Close();
    */



    // Printing
    cout << "Memory Address: " << dataPtr << endl;
    cout << "Size of dataPtr: " << sizeof(dataPtr) << endl;
    // For loop to iterate through the dataPtr -- only worry about first 3 elements of dataPtr
    for (int i = 0; i < 3; i++)
    {
        cout << "Structured Data: " << dataPtr[i] << endl;
    }
    cout << "Next point" << endl;


}


vector<Matrix<float, 3, 1> > SpotHandler::createJointTrajectory(Matrix<float, 3, 1> theta_list_start, Matrix<float, 3, 1> theta_list_end, float total_motion_time, int number_descrete_points, int time_scaling_method)
{
    Matrix<float, Dynamic, 3> dynamicTrajectory;
    dynamicTrajectory = myPath.createLegJointTrajectory(theta_list_start, theta_list_end, total_motion_time, number_descrete_points, time_scaling_method);

    cout << "Number of discrete points: " << number_descrete_points << endl;

    Matrix<float, 3, 1> dummyMat;
    vector<Matrix<float, 3, 1> > vectorMat;
    for (int i = 0; i < dynamicTrajectory.rows(); i++)
    {
        dummyMat[0] = dynamicTrajectory(i, 0);
        dummyMat[1] = dynamicTrajectory(i, 1);
        dummyMat[2] = dynamicTrajectory(i, 2);
        vectorMat.push_back(dummyMat);
    }
    return vectorMat;

}



int main()
{

    // *** Creating Parameters for Trajectory ***

    // Data that we want to work with
    Matrix<float, 3, 1> testStart;
    testStart << 0.0, 0.0, 0.0;

    Matrix<float, 3, 1> testEnd;
    testEnd << M_PI * 0.25, M_PI * 0.25, 1.5;

    float totalMotionTime = 5;      // seconds
    int numberDiscretePoints = 10;  // number of points in trajectory
    int timeScaling = 5;            // 5th degree polynomial


    // *** Sending Data *** 

    // File to test
    //string testFile = "testerData.dat";

    string testFile = "/dev/ttyACM1";

    SpotHandler myHandler;

    vector<Matrix<float, 3, 1> > vectorMat;
    vectorMat = myHandler.createJointTrajectory(testStart, testEnd, totalMotionTime, numberDiscretePoints, timeScaling);

    cout << "Initializing testing" << endl;

    myHandler.sendTrajectory(testFile, vectorMat);

    myHandler.readFromFile("testerData.dat");



    return 0;
}

