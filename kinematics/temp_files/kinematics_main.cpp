// using eigen -- https://gitlab.com/libeigen/eigen
// adding eigen -- use #include <Eigen/Dense> -- when compiling, provide path to eigen library -- eigen-3.4.0
// for me g++ -I ../eigen-3.4.0 file_name.cpp -o desired_compiled_file_name
// "../" is used because on my local computer, my eigen folder was in kinematics_tester.cpp's parent directory
// With this structure kinematics_tester.cpp should be in the same location as the eigen libraries

// more references for eigen -- https://aleksandarhaber.com/starting-with-eigen-c-matrix-library/

#include "kinematics.h"

#define _USE_MATH_DEFINES

#include <iostream>
#include <string>
#include <stdio.h> /* printf */
#include <math.h>  /* atan2 */
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Core>

#include <unsupported/Eigen/MatrixFunctions>

using Eigen::Matrix;
using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using namespace Eigen;

using namespace std;

#define PI 3.14159265


int main()
{

    // Creating DogKinematics object
    // DogKinematics doggo_1 = DogKinematics(1.0, 1.0, 0.1, 0.2, 0.2);
    DogKinematics doggo_1(1.0f, 1.0f, 0.1f, 0.2f, 0.2f);

    /*
    // Initilizing variables
    doggo_1.robot_length = 1.0;
    doggo_1.robot_width = 1.0;
    doggo_1.hip_length = 0.1;
    doggo_1.shoulder_length = 0.2;
    doggo_1.leg_length = 0.2;
    */

    // Initial condition -- joint angles all 0
    doggo_1.theta_list << M_PI / 4, M_PI / 2, M_PI / 4, M_PI / 4, M_PI / 4, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f;

    // chassis to hip transforms
    doggo_1.transform_m_to_front_right_0 << cos(PI / 2), 0, sin(PI / 2), doggo_1.robot_length / 2,
        0, 1, 0, 0,
        -sin(PI / 2), 0, cos(PI / 2), doggo_1.robot_width / 2,
        0, 0, 0, 1;

    doggo_1.transform_m_to_front_left_0 << cos(-PI / 2), 0, sin(-PI / 2), doggo_1.robot_length / 2,
        0, 1, 0, 0,
        -sin(-PI / 2), 0, cos(-PI / 2), -doggo_1.robot_width / 2,
        0, 0, 0, 1;

    doggo_1.transform_m_to_rear_right_0 << cos(PI / 2), 0, sin(PI / 2), -doggo_1.robot_length / 2,
        0, 1, 0, 0,
        -sin(PI / 2), 0, cos(PI / 2), doggo_1.robot_width / 2,
        0, 0, 0, 1;

    doggo_1.transform_m_to_rear_left_0 << cos(-PI / 2), 0, sin(-PI / 2), -doggo_1.robot_length / 2,
        0, 1, 0, 0,
        -sin(-PI / 2), 0, cos(-PI / 2), -doggo_1.robot_width / 2,
        0, 0, 0, 1;

    // Initilizing M, the home matrix
    doggo_1.home_mat << 0, 0, -1, -doggo_1.hip_length,
        -1, 0, 0, -(doggo_1.shoulder_length + doggo_1.leg_length),
        0, 1, 0, 0,
        0, 0, 0, 1;

    // Initilizing screw axes
    doggo_1.screw_1 << 0, 0, 1, 0, 0, 0;
    doggo_1.screw_2 << -1, 0, 0, 0, 0, 0;
    doggo_1.screw_3 << -1, 0, 1, 0, 0, -doggo_1.shoulder_length;

    // *** Printing ***
    Matrix<float, 4, 4> first_foot_transform;
    Matrix<float, 4, 4> second_foot_transform;
    Matrix<float, 12, 1> first_theta_list;
    Matrix<float, 12, 1> second_theta_list;

    first_theta_list = doggo_1.theta_list;

    // Testing FK and IK
    doggo_1.calculateForwardKinematics(doggo_1.theta_list);

    cout << "Transform from front right hip {0} to front right foot {4} is:\n"
         << doggo_1.trans_front_right_0_4 << endl;
    // cout << "Transform from front left hip {0} to front left foot {4} is:\n" << doggo_1.trans_front_left_0_4 << endl;
    // cout << "Transform from rear right hip {0} to rear right foot {4} is:\n" << doggo_1.trans_rear_right_0_4 << endl;
    // cout << "Transform from rear left hip {0} to rear left foot {4} is:\n" << doggo_1.trans_rear_left_0_4 << endl;

    first_foot_transform = doggo_1.trans_front_right_0_4;

    // doggo_1.findNewFK(doggo_1.theta_list);

    // cout << "Done a different way, the transform from front right hip {0} to front right foot {4} is:\n" << doggo_1.trans_front_right_0_4 << endl;

    // cout << "Done a different way, the transform from front right hip {0} to front left foot {4} is:\n" << doggo_1.trans_front_left_0_4 << endl;

    // cout << "Done a different way, the transform from front rear hip {0} to front right foot {4} is:\n" << doggo_1.trans_rear_right_0_4 << endl;

    // cout << "Done a different way, the transform from front rear hip {0} to front left foot {4} is:\n" << doggo_1.trans_rear_left_0_4 << endl;

    doggo_1.calculateInverseKinematics(doggo_1.trans_front_right_0_4, doggo_1.trans_front_left_0_4, doggo_1.trans_rear_right_0_4, doggo_1.trans_rear_left_0_4);

    cout << "Current theta_list:\n"
         << doggo_1.theta_list << endl;

    second_theta_list = doggo_1.theta_list;

    // transform = doggo_1.makeTransformMatrix(rotation, translation);

    /*
    cout << "The transform is:\n" << transform << endl;

    cout << "transform to front right 0 is:\n" << doggo_1.transform_m_to_front_right_0 << endl;
    cout << "transform to front left 0 is:\n" << doggo_1.transform_m_to_front_left_0 << endl;
    cout << "transform to rear right 0 is:\n" << doggo_1.transform_m_to_rear_right_0 << endl;
    cout << "transform to rear left 0 is:\n" << doggo_1.transform_m_to_rear_left_0 << endl;

    */

    doggo_1.calculateForwardKinematics(doggo_1.theta_list);

    cout << "Second loop through of the transform from front right hip {0} to front right foot {4} is:\n"
         << doggo_1.trans_front_right_0_4 << endl;

    second_foot_transform = doggo_1.trans_front_right_0_4;

    // printing out error between first and 2nd run through of FK
    cout << "\nError between 1st and 2nd run through of FK:\n"
         << second_foot_transform - first_foot_transform << endl;

    cout << "\nError between initial theta_list and theta_list after IK\n"
         << second_theta_list - first_theta_list << endl;
    return 0;
}