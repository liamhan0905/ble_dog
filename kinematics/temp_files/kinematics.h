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


#define PI 3.14159265

class DogKinematics
{
    // Constructor
public:
    // Constructor
    
    /*
    DogKinematics(float robot_length, float robot_width, float hip_length, float shoulder_length, float leg_length)
        : robot_length(robot_length), robot_width(robot_width), hip_length(hip_length), shoulder_length(shoulder_length), leg_length(leg_length) {}

    */

    DogKinematics(float robot_length, float robot_width, float hip_length, float shoulder_length, float leg_length);

    // Main chassis
    float robot_length;
    float robot_width;

    // Hip -- SHoulder -- Leg
    float hip_length;
    float shoulder_length;
    float leg_length;

    // Transforms from chassis (m) to each robot leg -- transform_m_to_front_right_0, transform_m_to_front_left_0,
    //                                                  transform_m_to_rear_right_0, transform_m_to_rear_left_0
    Matrix<float, 4, 4> transform_m_to_front_right_0;
    Matrix<float, 4, 4> transform_m_to_front_left_0;
    Matrix<float, 4, 4> transform_m_to_rear_right_0;
    Matrix<float, 4, 4> transform_m_to_rear_left_0;

    // ***   Transforms from frame 0 to frame 4 for each leg      ***
    // Format:
    //   0_1: Transform from hip joint to should joint along hip_length
    //   1_2: Transform that rotates from hip's z axis to shoulder's z axis
    //   2_3: Transform from shoulder joint to leg joint along shoulder_length
    //   3_4: Transform from leg joint to foot along leg_length
    // Don't actually need to have all of these for the 4 different legs, the differences in transformations are from the
    //   above transform_m_to_x matrices

    // Front Right Leg
    Matrix<float, 4, 4> trans_front_right_0_1;
    Matrix<float, 4, 4> trans_front_right_1_2;
    Matrix<float, 4, 4> trans_front_right_2_3;
    Matrix<float, 4, 4> trans_front_right_3_4;

    Matrix<float, 4, 4> trans_front_right_0_4;

    // Front Left Leg
    Matrix<float, 4, 4> trans_front_left_0_1;
    Matrix<float, 4, 4> trans_front_left_1_2;
    Matrix<float, 4, 4> trans_front_left_2_3;
    Matrix<float, 4, 4> trans_front_left_3_4;

    Matrix<float, 4, 4> trans_front_left_0_4;

    // Rear Right Leg
    Matrix<float, 4, 4> trans_rear_right_0_1;
    Matrix<float, 4, 4> trans_rear_right_1_2;
    Matrix<float, 4, 4> trans_rear_right_2_3;
    Matrix<float, 4, 4> trans_rear_right_3_4;

    Matrix<float, 4, 4> trans_rear_right_0_4;

    // Rear Left Leg
    Matrix<float, 4, 4> trans_rear_left_0_1;
    Matrix<float, 4, 4> trans_rear_left_1_2;
    Matrix<float, 4, 4> trans_rear_left_2_3;
    Matrix<float, 4, 4> trans_rear_left_3_4;

    Matrix<float, 4, 4> trans_rear_left_0_4;


    // X by 1 vector of joint angles -- from theta_1 to theta_12
    //   Format is:
    //     front right leg from 1 to 3 -- theta_1, theta_2, theta_3
    //     front left  leg from 1 to 3 -- theta_4, theta_5, theta_6
    //     rear  right leg from 1 to 3 -- theta_7, theta_8, theta_9
    //     rear  left  leg from 1 to 3 -- theta_10, theta_11, theta_12
    Matrix<float, 12, 1> theta_list;


    // Definition of M matrix, the home configuration of the end effector relative to the arm's {0} frame -- for T(theta), T(0) = M
    Matrix<float, 4, 4> home_mat;

    // Screw axis definitions, correspionding to the combined angular and linear velocities of each portion of a leg
    Matrix<float, 6, 1> screw_1;
    Matrix<float, 6, 1> screw_2;
    Matrix<float, 6, 1> screw_3;

    void calculateForwardKinematics(Matrix<float, 12, 1> theta_list_var);

    void calculateInverseKinematics(Matrix<float, 4, 4> front_right_foot_transform,
                                    Matrix<float, 4, 4> front_left_foot_transform,
                                    Matrix<float, 4, 4> rear_right_foot_transform,
                                    Matrix<float, 4, 4> rear_left_foot_transform);

    
};