
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

using Eigen::MatrixXd;
using Eigen::Matrix4d;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::Matrix;
using namespace Eigen;

using namespace std;

#define PI 3.14159265

/*

DogKinematics::DogKinematics(float robot_length, float robot_width, float hip_length, float shoulder_length, float leg_length)
    : robot_length(robot_length), robot_width(robot_width), hip_length(hip_length), shoulder_length(shoulder_length), leg_length(leg_length)
{
}

*/

DogKinematics::DogKinematics(float robot_length, float robot_width, float hip_length, float shoulder_length, float leg_length)
{
    // Main chassis
    robot_length = robot_length;
    robot_width = robot_width;

    // Hip -- SHoulder -- Leg
    hip_length = hip_length;
    shoulder_length = shoulder_length;
    leg_length = leg_length;
}


void DogKinematics::calculateForwardKinematics(Matrix<float, 12, 1> theta_list_var)
{

    //   From theta_list declaration, the format is:
    //     front right leg from 1 to 3 -- theta_1, theta_2, theta_3
    //     front left  leg from 1 to 3 -- theta_4, theta_5, theta_6
    //     rear  right leg from 1 to 3 -- theta_7, theta_8, theta_9
    //     rear  left  leg from 1 to 3 -- theta_10, theta_11, theta_12

    // Structure for forward kinematics:
    //   Set transforms for hip, shoulder, leg, and foot frames to appropriate angles within theta_list_var
    //   Multiply these transforms together to get T_0_4 -- T_0_4 = T_0_1 * T_1_2 * T_2_3 * T_3_4
    //     T_1_2 is a pure rotation -- in future I can mix it in with T_2_3
    //

    // can implement for loop once I figure out how to parse based on text of variable name
    // for now just assign values individually

    // Front Right Leg

    trans_front_right_0_1 << cos(theta_list_var[0]), -sin(theta_list_var[0]), 0, -hip_length * cos(theta_list_var[0]),
        sin(theta_list_var[0]), cos(theta_list_var[0]), 0, -hip_length * sin(theta_list_var[0]),
        0, 0, 1, 0,
        0, 0, 0, 1;
    trans_front_right_1_2 << 0, 0, -1, 0,
        -1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 0, 1;
    trans_front_right_2_3 << cos(theta_list_var[1]), -sin(theta_list_var[1]), 0, shoulder_length * cos(theta_list_var[1]),
        sin(theta_list_var[1]), cos(theta_list_var[1]), 0, shoulder_length * sin(theta_list_var[1]),
        0, 0, 1, 0,
        0, 0, 0, 1;
    trans_front_right_3_4 << cos(theta_list_var[2]), -sin(theta_list_var[2]), 0, leg_length * cos(theta_list_var[2]),
        sin(theta_list_var[2]), cos(theta_list_var[2]), 0, leg_length * sin(theta_list_var[2]),
        0, 0, 1, 0,
        0, 0, 0, 1;

    trans_front_right_0_4 = trans_front_right_0_1 * trans_front_right_1_2 * trans_front_right_2_3 * trans_front_right_3_4;

    // Front Left Leg
    trans_front_left_0_1 << cos(theta_list_var[3]), -sin(theta_list_var[3]), 0, -hip_length * cos(theta_list_var[3]),
        sin(theta_list_var[3]), cos(theta_list_var[3]), 0, -hip_length * sin(theta_list_var[3]),
        0, 0, 1, 0,
        0, 0, 0, 1;
    trans_front_left_1_2 << 0, 0, -1, 0,
        -1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 0, 1;
    trans_front_left_2_3 << cos(theta_list_var[4]), -sin(theta_list_var[4]), 0, shoulder_length * cos(theta_list_var[4]),
        sin(theta_list_var[4]), cos(theta_list_var[4]), 0, shoulder_length * sin(theta_list_var[4]),
        0, 0, 1, 0,
        0, 0, 0, 1;
    trans_front_left_3_4 << cos(theta_list_var[5]), -sin(theta_list_var[5]), 0, leg_length * cos(theta_list_var[5]),
        sin(theta_list_var[5]), cos(theta_list_var[5]), 0, leg_length * sin(theta_list_var[5]),
        0, 0, 1, 0,
        0, 0, 0, 1;

    trans_front_left_0_4 = trans_front_left_0_1 * trans_front_left_1_2 * trans_front_left_2_3 * trans_front_left_3_4;

    // Rear Right Leg
    trans_rear_right_0_1 << cos(theta_list_var[6]), -sin(theta_list_var[6]), 0, -hip_length * cos(theta_list_var[6]),
        sin(theta_list_var[6]), cos(theta_list_var[6]), 0, -hip_length * sin(theta_list_var[6]),
        0, 0, 1, 0,
        0, 0, 0, 1;
    trans_rear_right_1_2 << 0, 0, -1, 0,
        -1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 0, 1;
    trans_rear_right_2_3 << cos(theta_list_var[7]), -sin(theta_list_var[7]), 0, shoulder_length * cos(theta_list_var[7]),
        sin(theta_list_var[7]), cos(theta_list_var[7]), 0, shoulder_length * sin(theta_list_var[7]),
        0, 0, 1, 0,
        0, 0, 0, 1;
    trans_rear_right_3_4 << cos(theta_list_var[8]), -sin(theta_list_var[8]), 0, leg_length * cos(theta_list_var[8]),
        sin(theta_list_var[8]), cos(theta_list_var[8]), 0, leg_length * sin(theta_list_var[8]),
        0, 0, 1, 0,
        0, 0, 0, 1;

    trans_rear_right_0_4 = trans_rear_right_0_1 * trans_rear_right_1_2 * trans_rear_right_2_3 * trans_rear_right_3_4;

    // Rear Left Leg
    trans_rear_left_0_1 << cos(theta_list_var[9]), -sin(theta_list_var[9]), 0, -hip_length * cos(theta_list_var[9]),
        sin(theta_list_var[9]), cos(theta_list_var[9]), 0, -hip_length * sin(theta_list_var[9]),
        0, 0, 1, 0,
        0, 0, 0, 1;
    trans_rear_left_1_2 << 0, 0, -1, 0,
        -1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 0, 1;
    trans_rear_left_2_3 << cos(theta_list_var[10]), -sin(theta_list_var[10]), 0, shoulder_length * cos(theta_list_var[10]),
        sin(theta_list_var[10]), cos(theta_list_var[10]), 0, shoulder_length * sin(theta_list_var[10]),
        0, 0, 1, 0,
        0, 0, 0, 1;
    trans_rear_left_3_4 << cos(theta_list_var[11]), -sin(theta_list_var[11]), 0, leg_length * cos(theta_list_var[11]),
        sin(theta_list_var[11]), cos(theta_list_var[11]), 0, leg_length * sin(theta_list_var[11]),
        0, 0, 1, 0,
        0, 0, 0, 1;

    trans_rear_left_0_4 = trans_rear_left_0_1 * trans_rear_left_1_2 * trans_rear_left_2_3 * trans_rear_left_3_4;

    cout << "I did FK!" << endl;
}

void DogKinematics::calculateInverseKinematics(Matrix<float, 4, 4> front_right_foot_transform,
                                          Matrix<float, 4, 4> front_left_foot_transform,
                                          Matrix<float, 4, 4> rear_right_foot_transform,
                                          Matrix<float, 4, 4> rear_left_foot_transform)
{
    // Reference
    // Goal is to calculate different joint angles based on the position of the reference frame of the foot (ie of reference frame {4}) for each leg.
    // Since each foot transform is a 4x4 matrix describing a combined transformation and rotation, the position of the foot transform relative to the
    //   base frame {0} for a particular leg is given by the bracketed indexes:
    //   [foot00, foot01, foot02, { foot03 }],
    //   [foot10, foot11, foot12, { foot13 }],
    //   [foot20, foot21, foot22, { foot23 }],
    //   [foot30, foot31, foot32, foot33]
    // foot33 should be 1

    // Front Right
    float front_right_foot_x = front_right_foot_transform(0, 3);
    float front_right_foot_y = front_right_foot_transform(1, 3);
    float front_right_foot_z = front_right_foot_transform(2, 3);

    cout << "front_right_foot_x: " << front_right_foot_x << endl;
    cout << "front_right_foot_y: " << front_right_foot_y << endl;
    cout << "front_right_foot_z: " << front_right_foot_z << endl;

    // Front Left
    float front_left_foot_x = front_left_foot_transform(0, 3);
    float front_left_foot_y = front_left_foot_transform(1, 3);
    float front_left_foot_z = front_left_foot_transform(2, 3);

    // Rear Right
    float rear_right_foot_x = rear_right_foot_transform(0, 3);
    float rear_right_foot_y = rear_right_foot_transform(1, 3);
    float rear_right_foot_z = rear_right_foot_transform(2, 3);

    // Rear Left
    float rear_left_foot_x = rear_left_foot_transform(0, 3);
    float rear_left_foot_y = rear_left_foot_transform(1, 3);
    float rear_left_foot_z = rear_left_foot_transform(2, 3);

    // Declaring variables to hold different components -- meant to iterate through
    Matrix<float, 4, 1> foot_x;
    Matrix<float, 4, 1> foot_y;
    Matrix<float, 4, 1> foot_z;

    // Initilizing
    foot_x << front_right_foot_x, front_left_foot_x, rear_right_foot_x, rear_left_foot_x;
    foot_y << front_right_foot_y, front_left_foot_y, rear_right_foot_y, rear_left_foot_y;
    foot_z << front_right_foot_z, front_left_foot_z, rear_right_foot_z, rear_left_foot_z;

    // Creating vecctor that's just positive in the xhat0 direction, since I'm blanking on the given value for it
    Matrix<float, 3, 1> x_direction;
    x_direction << 1, 0, 0;

    // Creating vector that's positive in the y direction
    Matrix<float, 3, 1> y_direction;
    y_direction << 0, 1, 0;

    // Creating vector that's positive in the z direction
    Matrix<float, 3, 1> z_direction;
    z_direction << 0, 0, 1;

    // Iterating through each leg
    for (int leg_iterator = 0; leg_iterator < 4; leg_iterator++)
    {
        // Establishing the translation vector from framne {0} to frame {4}; this changes as you iterate through legs 1 through 4
        Vector3f p_0_4(foot_x[leg_iterator], foot_y[leg_iterator], foot_z[leg_iterator]);
        // Establishing the magnitude of p_0_4
        Matrix<float, 3, 1> p_0_4_prime;
        p_0_4_prime << p_0_4(0), p_0_4(1), 0;

        float p_4_magnitude = p_0_4.norm();

        float p_4_magnitude_prime = p_0_4_prime.norm();

        float tau = sqrt(p_4_magnitude * p_4_magnitude - hip_length * hip_length);
        float tau_prime = sqrt(p_4_magnitude_prime * p_4_magnitude_prime - hip_length * hip_length);

        // Creating u_vec and v_vec, two vectors defining the path from p1 to p3
        // v_vec can be calculated with Heron's Formula -- here 1/2 * base * height = () Heron's formula ),
        //   where base is tau, height is |v_vec|, and s is 1/2 * (shoulder_length + leg_length + tau).
        float s_heron = 0.5 * (shoulder_length + leg_length + tau);
        float v_vec_magnitude = (2 / tau) * sqrt(abs(s_heron * (s_heron - shoulder_length) * (s_heron - leg_length) * (s_heron - tau)));

        // cout << "\nmagnitude of v_vec: " << v_vec_magnitude << endl;

        // u_vec can be found through its magnitude and direction. It's magnitude can be found through pythagorean's theorem with u_vec, v_vec, and shoulder_length

        float u_vec_magnitude = sqrt(shoulder_length * shoulder_length - v_vec_magnitude * v_vec_magnitude);

        // Defining phi_angle to help calculate theta 3 -- phi_angle and theta 3 are supplementary (add to PI)
        float acos_input_for_phi = (shoulder_length * shoulder_length + leg_length * leg_length - tau * tau) / (2 * shoulder_length * leg_length);
        float phi_angle = acos(acos_input_for_phi);

        float alpha_angle = asin(tau_prime / p_4_magnitude_prime);

        float beta_angle = acos(p_0_4_prime.dot(x_direction) / p_4_magnitude_prime);

        // Changing angle values

        // Theta 1, theta 4, theta 7, and theta 10 each depend on the quadrant they're located in
        // Craft if statement blocks according to position of p_0_4

        //     2  |  1
        //    ----.----
        //     3  |  4

        /*
        if (foot_x[leg_iterator] <= 0 && foot_y[leg_iterator] <= 0)
        {
            // We're in the lower left quadrant, quadrant 3
            theta_list[0 + 3 * leg_iterator] = PI - alpha_angle - beta_angle;

            if (leg_iterator == 0)
            {
                cout << "\nTheta 0 quad 3: " << theta_list[0 + 3 * leg_iterator] << endl;
            }

        }
        else if (foot_x[leg_iterator] <= 0 && foot_y[leg_iterator] > 0)
        {


            // We're in the upper left quadrant, quadrant 2
            theta_list[0 + 3 * leg_iterator] = alpha_angle + (PI - beta_angle);



            if (leg_iterator == 0)
            {
                cout << "\nTheta 0 quad 2: " << theta_list[0 + 3 * leg_iterator] << endl;
            }


        }
        else if (foot_x[leg_iterator] > 0 && foot_y[leg_iterator] > 0)
        {
            // We're in the upper right quadrant, quadrant 1
            theta_list[0 + 3 * leg_iterator] = alpha_angle + (PI - beta_angle);

            if (leg_iterator == 0)
            {
                cout << "\nTheta 0 quad 1: " << theta_list[0 + 3 * leg_iterator] << endl;
            }


        }
        else
        {
            // We're in the lower right quadrant, quadrant 4
            theta_list[0 + 3 * leg_iterator] = PI - alpha_angle - beta_angle;

            if (leg_iterator == 0)
            {
                cout << "\nTheta 0 quad 4: " << theta_list[0 + 3 * leg_iterator] << endl;
            }
        }
        */

        theta_list[0 + 3 * leg_iterator] = (alpha_angle + beta_angle - M_PI) * (p_0_4(1) / abs(p_0_4(1)));

        Matrix<float, 3, 1> p_0_1;
        p_0_1 << -hip_length * cos(theta_list[0 + 3 * leg_iterator]), -hip_length * sin(theta_list[0 + 3 * leg_iterator]), 0;

        Matrix<float, 3, 1> u_vec;
        // float p_to_u_factor = u_vec_magnitude / p_4_magnitude; -- this was for the previuus formulation of u_vec, which said that u_vec and p_0_4 point in
        //   the same direction. This is not the case, u_vec points in the direction of p_0_4 - p_0_1
        float p_to_u_factor = u_vec_magnitude / tau;

        u_vec = p_to_u_factor * (p_0_4 - p_0_1);

        // reassigning u_vec_magnitude?
        u_vec_magnitude = u_vec.norm();

        Matrix<float, 3, 1> v_vec;
        v_vec = v_vec_magnitude * u_vec.cross(p_0_1);

        // Creating p_0_3, in terms of u_vec and v_vec (which are in turn based on p_0_4) -- p_0_3 = p_0_1 + u_vec + v_vec;
        Matrix<float, 3, 1> p_0_3;
        p_0_3 = p_0_1 + u_vec + v_vec;
        float p_3_magnitude = p_0_3.norm();

        if (leg_iterator == 0)
        {
            cout << "p_0_1:\n"
                 << p_0_1 << "\nu_vec:\n"
                 << u_vec << "\nv_vec:\n"
                 << v_vec << endl;

            cout << "p_0_3:\n"
                 << p_0_3
                 << "\np_3 - p_1\n"
                 << p_0_3 - p_0_1
                 << "\n p_3 - p_1 dotted with u_vec\n"
                 << (p_0_3 - p_0_1).dot(u_vec)
                 << "\n u_vec + v_vec dotted with u_vec\n"
                 << (u_vec + v_vec).dot(u_vec)
                 << "\nu_vec_magnitude * shoulder_length:\n"
                 << u_vec_magnitude * shoulder_length << endl;

            cout << "\natan2(v_vec, u_vec):\n"
                 << atan2(v_vec_magnitude, u_vec_magnitude) << endl;

            cout << "\nTesting to see if tau is the same as the magnitude of p_4 - p_1\n"
                 << "\ntau\n"
                 << tau
                 << "\n magnitude of p_4 - p_1\n"
                 << (p_0_4 - p_0_1).norm() << endl;
        }

        // Create a storage variable for theta 3, such that you can compare the error between new and old theta values
        float leg_angle_error_tolerance = 0.000001;

        theta_list[2 + 3 * leg_iterator] = M_PI - phi_angle;

        // Angles to help calculate theta 2
        float gamma_angle;
        float xhi_angle;

        // Generating cross product of z_direction and p_0_1
        Matrix<float, 3, 1> zhat_cros_p1;
        zhat_cros_p1 = z_direction.cross(p_0_1);
        float zhat_cros_p1_magnitude = zhat_cros_p1.norm();

        gamma_angle = acos((float)(p_0_4 - p_0_1).dot(zhat_cros_p1) / (tau * zhat_cros_p1_magnitude));

        xhi_angle = acos((float)(p_0_3 - p_0_1).dot(u_vec) / (shoulder_length * u_vec_magnitude));

        theta_list[1 + 3 * leg_iterator] = gamma_angle - xhi_angle;

        if (leg_iterator == 0)
        {

            cout << "\ngamma_angle: " << gamma_angle << endl;
            cout << "\nxhi_angle: " << xhi_angle << endl;

            cout << "\nTheta 2: " << theta_list[1 + 3 * leg_iterator] << endl;
        }
    }

    cout << "I did IK!" << endl;
}