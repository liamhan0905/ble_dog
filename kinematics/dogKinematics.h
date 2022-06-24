
// using eigen -- https://gitlab.com/libeigen/eigen
// adding eigen -- use #include <Eigen/Dense> -- when compiling, provide path to eigen library -- eigen-3.4.0
// for me g++ -I ../eigen-3.4.0 file_name.cpp -o desired_compiled_file_name
// "../" is used because on my local computer, my eigen folder was in kinematics_tester.cpp's parent directory
// With this structure kinematics_tester.cpp should be in the same location as the eigen libraries

// more references for eigen -- https://aleksandarhaber.com/starting-with-eigen-c-matrix-library/



#define _USE_MATH_DEFINES


#include <iostream>
#include <string>
#include <stdio.h> /* printf */
#include <math.h>  /* atan2 */
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Core>

#include <unsupported/Eigen/MatrixFunctions>

using Eigen::MatrixXd;
using Eigen::Matrix4d;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::Matrix;
using namespace Eigen;

using namespace std;

#define PI 3.14159265


// Calculates forward and inverse kinematics for a 4 legged mini spot robot, according to this documentation:
//https: // www.ijstr.org/final-print/sep2017/Inverse-Kinematic-Analysis-Of-A-Quadruped-Robot.pdf

// Define class for dog kinematics
class DogKinematics
{
    // Constructor
    public:
        // Constructor
        
        DogKinematics(float robot_length, float robot_width, float hip_length, float shoulder_length, float leg_length, Matrix<float, 12, 1> theta_list)
            : robot_length(robot_length)
            , robot_width(robot_width)
            , hip_length(hip_length)
            , shoulder_length(shoulder_length)
            , leg_length(leg_length)
            , theta_list(theta_list)
        {
            cout << "robot_length: " << robot_length << "\nrobot_width: " << robot_width << "\nhip_length: " << hip_length << "\nshoulder_length: " << shoulder_length << "\nleg_length: " << leg_length << endl;
            cout << "Theta List:\n" << theta_list << endl;

            transform_body.setIdentity();

            transform_m_to_front_right_0 << cos(PI / 2), 0, sin(PI / 2), robot_length / 2,
                0, 1, 0, 0,
                -sin(PI / 2), 0, cos(PI / 2), robot_width / 2,
                0, 0, 0, 1;

            transform_m_to_front_left_0 << cos(-PI / 2), 0, sin(-PI / 2), robot_length / 2,
                0, 1, 0, 0,
                -sin(-PI / 2), 0, cos(-PI / 2), -robot_width / 2,
                0, 0, 0, 1;

            transform_m_to_rear_right_0 << cos(PI / 2), 0, sin(PI / 2), -robot_length / 2,
                0, 1, 0, 0,
                -sin(PI / 2), 0, cos(PI / 2), robot_width / 2,
                0, 0, 0, 1;

            transform_m_to_rear_left_0 << cos(-PI / 2), 0, sin(-PI / 2), -robot_length / 2,
                0, 1, 0, 0,
                -sin(-PI / 2), 0, cos(-PI / 2), -robot_width / 2,
                0, 0, 0, 1;


            calculateForwardKinematics(theta_list);

        }
        


        // Main chassis
        float robot_length;
        float robot_width;

        // Hip -- SHoulder -- Leg
        float hip_length;
        float shoulder_length;
        float leg_length;

        // Transform of the chassis -- represents the location of the origin -- initilized as I the identity matrix
        Matrix<float, 4, 4> transform_body;
        

        // ***   Transforms from frame 0 to frame 4 for each leg      ***
        // Format:
        //   0_1: Transform from hip joint to should joint along hip_length
        //   1_2: Transform that rotates from hip's z axis to shoulder's z axis
        //   2_3: Transform from shoulder joint to leg joint along shoulder_length
        //   3_4: Transform from leg joint to foot along leg_length

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

        // *** End of Transforms from frame 0 to frame 4 for each leg ***


        // X by 1 vector of joint angles -- from theta_1 to theta_12
        //   Format is:
        //     front right leg from 1 to 3 -- theta_1, theta_2, theta_3
        //     front left  leg from 1 to 3 -- theta_4, theta_5, theta_6
        //     rear  right leg from 1 to 3 -- theta_7, theta_8, theta_9
        //     rear  left  leg from 1 to 3 -- theta_10, theta_11, theta_12
        Matrix<float, 12, 1> theta_list;


        // Definition of M matrix, the home configuration of the end effector -- for T(theta), T(0) = M
        Matrix<float, 4, 4> home_mat;

        // Screw axis definitions, correspionding to the combined angular and linear velocities of each portion of a leg
        Matrix<float, 6, 1> screw_1;
        Matrix<float, 6, 1> screw_2;
        Matrix<float, 6, 1> screw_3;

        // Transforms from chassis (m) to each robot leg -- transform_m_to_front_right_0, transform_m_to_front_left_0,
        //                                                  transform_m_to_rear_right_0, transform_m_to_rear_left_0
        Matrix<float, 4, 4> transform_m_to_front_right_0;
        Matrix<float, 4, 4> transform_m_to_front_left_0;
        Matrix<float, 4, 4> transform_m_to_rear_right_0;
        Matrix<float, 4, 4> transform_m_to_rear_left_0;

        // Member Functions

        // different formulation of forward kinematics -- only implemented for the front right leg
        void findNewFK(Matrix<float, 12, 1> theta_list_var);

        void calculateForwardKinematics(Matrix<float, 12, 1> theta_list_var);

        void calculateInverseKinematics(Matrix<float, 4, 4> front_right_foot_transform,
                                Matrix<float, 4, 4> front_left_foot_transform,
                                Matrix<float, 4, 4> rear_right_foot_transform,
                                Matrix<float, 4, 4> rear_left_foot_transform);

        Matrix<float, 4, 4> makeTransformMatrix(Matrix<float, 3, 3> rotation, Matrix<float, 3, 1> translation);

        Matrix<float, 4, 4> makeSkewSymmetricScrewMatrix(Matrix<float, 6, 1> screw_axis);


        void moveBody(Matrix<float, 4, 4> body_change);

        void moveFrontRightFoot(Matrix<float, 4, 4> foot_change);
        void moveFrontLeftFoot(Matrix<float, 4, 4> foot_change);
        void moveRearRightFoot(Matrix<float, 4, 4> foot_change);
        void moveRearLeftFoot(Matrix<float, 4, 4> foot_change);


        // *** Accessor Functions ***

        Matrix<float, 4, 4> getBodyTransform();

        Matrix<float, 12, 1> getThetaList();

        Matrix<float, 4, 4> getFrontRight();
        Matrix<float, 4, 4> getFrontLeft();
        Matrix<float, 4, 4> getRearRight();
        Matrix<float, 4, 4> getRearLeft();

        // *** End of Accessor Functions ***

};