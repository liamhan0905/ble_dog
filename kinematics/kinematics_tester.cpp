
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
        DogKinematics(float robot_length, float robot_width, float hip_length, float shoulder_length, float leg_length)
            : robot_length(robot_length)
            , robot_width(robot_width)
            , hip_length(hip_length)
            , shoulder_length(shoulder_length)
            , leg_length(leg_length)
        {
            cout << "robot_length: " << robot_length << "\nrobot_width: " << robot_width << "\nhip_length: " << hip_length << "\nshoulder_length: " << shoulder_length << "\nleg_length: " << leg_length << endl;
        }

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

        // *** End of Transforms from frame 0 to frame 4 for each leg ***


        //Matrix<float, 4, 4> transform_m_to_front_right_0;
        //transform_m_to_front_right_0(0, 0) = cos(PI / 2);
        //Matrix<float, 4, 4> T;
        //T(0,0) = cos(PI/2);

        // X by 1 vector of joint angles -- from theta_1 to theta_12
        //   Format is:
        //     front right leg from 1 to 3 -- theta_1, theta_2, theta_3
        //     front left  leg from 1 to 3 -- theta_4, theta_5, theta_6
        //     rear  right leg from 1 to 3 -- theta_7, theta_8, theta_9
        //     rear  left  leg from 1 to 3 -- theta_10, theta_11, theta_12
        Matrix<float, 12, 1> theta_list;


        
        // Structure for coordinate frames
        //   robot frame             : [main_x         , main_y         , main_z         ]
        //   front right arm frame 0 : [front_right_0_x, front_right_0_y, front_right_0_z]
        //   front right arm frame 1 : [front_right_1_x, front_right_1_y, front_right_1_z]
        //   front right arm frame 2 : [front_right_2_x, front_right_2_y, front_right_2_z]
        //   front right arm frame 3 : [front_right_3_x, front_right_3_y, front_right_3_z]
        //   front right arm frame 4 : [front_right_4_x, front_right_4_y, front_right_4_z]

        //   front left arm frame 0 : [front_left_0_x, front_left_0_y, front_left_0_z]
        //   front left arm frame 1 : [front_left_1_x, front_left_1_y, front_left_1_z]
        //   front left arm frame 2 : [front_left_2_x, front_left_2_y, front_left_2_z]
        //   front left arm frame 3 : [front_left_3_x, front_left_3_y, front_left_3_z]
        //   front left arm frame 4 : [front_left_4_x, front_left_4_y, front_left_4_z]

        //   rear right arm frame 0 : [rear_right_0_x, rear_right_0_y, rear_right_0_z]
        //   rear right arm frame 1 : [rear_right_1_x, rear_right_1_y, rear_right_1_z]
        //   rear right arm frame 2 : [rear_right_2_x, rear_right_2_y, rear_right_2_z]
        //   rear right arm frame 3 : [rear_right_3_x, rear_right_3_y, rear_right_3_z]
        //   rear right arm frame 4 : [rear_right_4_x, rear_right_4_y, rear_right_4_z]

        //   rear left arm frame 0 : [rear_left_0_x, rear_left_0_y, rear_left_0_z]
        //   rear left arm frame 0 : [rear_left_1_x, rear_left_1_y, rear_left_1_z]
        //   rear left arm frame 0 : [rear_left_2_x, rear_left_2_y, rear_left_2_z]
        //   rear left arm frame 0 : [rear_left_3_x, rear_left_3_y, rear_left_3_z]
        //   rear left arm frame 0 : [rear_left_4_x, rear_left_4_y, rear_left_4_z]

        // Definition of M matrix, the home configuration of the end effector -- for T(theta), T(0) = M
        Matrix<float, 4, 4> home_mat;

        // Screw axis definitions, correspionding to the combined angular and linear velocities of each portion of a leg
        Matrix<float, 6, 1> screw_1;
        Matrix<float, 6, 1> screw_2;
        Matrix<float, 6, 1> screw_3;


    // Member Functions


    // different formulation of forward kinematics -- only implemented for the front right leg
    void findNewFK(Matrix<float, 12, 1> theta_list_var)
    {
        // Defining skew-symetric matrices to prepare for taking the matrix exponential
        // makeSkewSymmetricScrewMatrix

        Matrix<float, 4, 4> skew1_theta1;
        Matrix<float, 4, 4> skew2_theta2;
        Matrix<float, 4, 4> skew3_theta3;

        skew1_theta1 = makeSkewSymmetricScrewMatrix(screw_1 * theta_list_var[0]);
        skew2_theta2 = makeSkewSymmetricScrewMatrix(screw_2 * theta_list_var[1]);
        skew3_theta3 = makeSkewSymmetricScrewMatrix(screw_3 * theta_list_var[2]);

        // Declaring matrix exponentials
        Matrix<float, 4, 4> e_S1_theta1;
        Matrix<float, 4, 4> e_S2_theta2;
        Matrix<float, 4, 4> e_S3_theta3;

        // Calculating and initilizing matrix exponentials
        e_S1_theta1 = skew1_theta1.exp();
        e_S2_theta2 = skew2_theta2.exp();
        e_S3_theta3 = skew3_theta3.exp();


        // Calculating and initilizing each leg's transform from frame 0 to frame 4 (hip to foot transform)
        trans_front_right_0_4 = e_S1_theta1 * e_S2_theta2 * e_S3_theta3 * home_mat;


        cout << "I did FK a different way!" << endl;
    }

    // given a set of joint angles, calculate and populate the object instance's foot transforms relative to its hip transform (its 0 frames for each leg)
    void calculateForwardKinematics(Matrix<float, 12, 1> theta_list_var)
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

        trans_front_right_0_1 <<
            cos(theta_list_var[0]), -sin(theta_list_var[0]), 0, -hip_length * cos(theta_list_var[0]),
            sin(theta_list_var[0]), cos(theta_list_var[0]) , 0, -hip_length * sin(theta_list_var[0]),
            0                     , 0                      , 1,                                    0,
            0                     , 0                      , 0,                                    1;
        trans_front_right_1_2 <<
            0, 0, -1, 0,
            -1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 0, 1;
        trans_front_right_2_3 << 
            cos(theta_list_var[1]), -sin(theta_list_var[1]), 0, shoulder_length * cos(theta_list_var[1]),
            sin(theta_list_var[1]), cos(theta_list_var[1]) , 0, shoulder_length * sin(theta_list_var[1]),
            0                     , 0                      , 1,                                        0,
            0                     , 0                      , 0,                                        1;
        trans_front_right_3_4 <<
            cos(theta_list_var[2]), -sin(theta_list_var[2]), 0, leg_length * cos(theta_list_var[2]),
            sin(theta_list_var[2]), cos(theta_list_var[2]) , 0, leg_length * sin(theta_list_var[2]),
            0                     , 0                      , 1,                                   0,
            0                     , 0                      , 0,                                   1;

        trans_front_right_0_4 = trans_front_right_0_1 * trans_front_right_1_2 * trans_front_right_2_3 * trans_front_right_3_4;

        // Front Left Leg
        trans_front_left_0_1 <<
            cos(theta_list_var[3]), -sin(theta_list_var[3]), 0, -hip_length * cos(theta_list_var[3]),
            sin(theta_list_var[3]), cos(theta_list_var[3]), 0, -hip_length * sin(theta_list_var[3]),
            0, 0, 1, 0,
            0, 0, 0, 1;
        trans_front_left_1_2 <<
            0, 0, -1, 0,
            -1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 0, 1;
        trans_front_left_2_3 <<
            cos(theta_list_var[4]), -sin(theta_list_var[4]), 0, shoulder_length * cos(theta_list_var[4]),
            sin(theta_list_var[4]), cos(theta_list_var[4]), 0, shoulder_length * sin(theta_list_var[4]),
            0, 0, 1, 0,
            0, 0, 0, 1;
        trans_front_left_3_4 <<
            cos(theta_list_var[5]), -sin(theta_list_var[5]), 0, leg_length * cos(theta_list_var[5]),
            sin(theta_list_var[5]), cos(theta_list_var[5]), 0, leg_length * sin(theta_list_var[5]),
            0, 0, 1, 0,
            0, 0, 0, 1;

        trans_front_left_0_4 = trans_front_left_0_1 * trans_front_left_1_2 * trans_front_left_2_3 * trans_front_left_3_4;

        // Rear Right Leg
        trans_rear_right_0_1 <<
            cos(theta_list_var[6]), -sin(theta_list_var[6]), 0, -hip_length * cos(theta_list_var[6]),
            sin(theta_list_var[6]), cos(theta_list_var[6]), 0, -hip_length * sin(theta_list_var[6]),
            0, 0, 1, 0,
            0, 0, 0, 1;
        trans_rear_right_1_2 <<
            0, 0, -1, 0,
            -1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 0, 1;
        trans_rear_right_2_3 <<
            cos(theta_list_var[7]), -sin(theta_list_var[7]), 0, shoulder_length * cos(theta_list_var[7]),
            sin(theta_list_var[7]), cos(theta_list_var[7]), 0, shoulder_length * sin(theta_list_var[7]),
            0, 0, 1, 0,
            0, 0, 0, 1;
        trans_rear_right_3_4 <<
            cos(theta_list_var[8]), -sin(theta_list_var[8]), 0, leg_length * cos(theta_list_var[8]),
            sin(theta_list_var[8]), cos(theta_list_var[8]), 0, leg_length * sin(theta_list_var[8]),
            0, 0, 1, 0,
            0, 0, 0, 1;

        trans_rear_right_0_4 = trans_rear_right_0_1 * trans_rear_right_1_2 * trans_rear_right_2_3 * trans_rear_right_3_4;

        // Rear Left Leg
        trans_rear_left_0_1 <<
            cos(theta_list_var[9]), -sin(theta_list_var[9]), 0, -hip_length * cos(theta_list_var[9]),
            sin(theta_list_var[9]), cos(theta_list_var[9]), 0, -hip_length * sin(theta_list_var[9]),
            0, 0, 1, 0,
            0, 0, 0, 1;
        trans_rear_left_1_2 <<
            0, 0, -1, 0,
            -1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 0, 1;
        trans_rear_left_2_3 <<
            cos(theta_list_var[10]), -sin(theta_list_var[10]), 0, shoulder_length * cos(theta_list_var[10]),
            sin(theta_list_var[10]), cos(theta_list_var[10]), 0, shoulder_length * sin(theta_list_var[10]),
            0, 0, 1, 0,
            0, 0, 0, 1;
        trans_rear_left_3_4 <<
            cos(theta_list_var[11]), -sin(theta_list_var[11]), 0, leg_length * cos(theta_list_var[11]),
            sin(theta_list_var[11]), cos(theta_list_var[11]), 0, leg_length * sin(theta_list_var[11]),
            0, 0, 1, 0,
            0, 0, 0, 1;

        trans_rear_left_0_4 = trans_rear_left_0_1 * trans_rear_left_1_2 * trans_rear_left_2_3 * trans_rear_left_3_4;


        cout << "I did FK!" << endl;

    }


    // given a set of coordinates foot_x, foot_y, and foot_z, describing the position of the foot for a particular leg, 
    //   calculate a possible set of joint angles theta1, theta2, and theta3 for that same leg that will bring the leg
    //   to the desired position
    // given a homogeneous transform X existing in SE(3), find solutions theta that satisfy T(theta) = X
    void calculateInverseKinematics(Matrix<float, 4, 4> front_right_foot_transform,
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



        // Iterating through each leg
        for (int leg_iterator = 0; leg_iterator < 4; leg_iterator++)
        {
            // Establishing the translation vector from framne {0} to frame {4}; this changes as you iterate through legs 1 through 4
            Vector3f p_0_4(foot_x[leg_iterator], foot_y[leg_iterator], foot_z[leg_iterator]);
            // Establishing the magnitude of p_0_4
            float p_4_magnitude = p_0_4.norm();

            // Creating vecctor that's just positive in the xhat0 direction, since I'm blanking on the given value for it
            Matrix<float, 3, 1> horizontal_direction;
            horizontal_direction << 1, 0, 0;

            // Creating vector that's positive in the y direction
            Matrix<float, 3, 1> y_direction;
            y_direction << 0, 1, 0;

            // Creating vector that's positive in the z direction
            Matrix<float, 3, 1> z_direction;
            z_direction << 0, 0, 1;

            float tau = sqrt(p_4_magnitude * p_4_magnitude - hip_length * hip_length);


            // Creating u_vec and v_vec, two vectors defining the path from p1 to p3
            // v_vec can be calculated with Heron's Formula -- here 1/2 * base * height = () Heron's formula ),
            //   where base is tau, height is |v_vec|, and s is 1/2 * (shoulder_length + leg_length + tau).
            float s_heron = 0.5 * (shoulder_length + leg_length + tau);
            float v_vec_magnitude = (2 / tau) * sqrt( abs( s_heron * (s_heron - shoulder_length) * (s_heron - leg_length) * (s_heron - tau) ) );



            // u_vec can be found through its magnitude and direction. It's magnitude can be found through pythagorean's theorem with u_vec, v_vec, and shoulder_length

            float u_vec_magnitude = sqrt(shoulder_length * shoulder_length - v_vec_magnitude * v_vec_magnitude);



            // Defining phi_angle to help calculate theta 3 -- phi_angle and theta 3 are supplementary (add to PI)
            float acos_input_for_phi = (shoulder_length * shoulder_length + leg_length * leg_length - tau * tau) / (2 * shoulder_length * leg_length);
            float phi_angle = acos(acos_input_for_phi);


            float alpha_angle = asin(tau / p_4_magnitude);

            float beta_angle = acos(p_0_4.dot(horizontal_direction) / p_4_magnitude );

            // Changing angle values

            // Theta 1, theta 4, theta 7, and theta 10 each depend on the quadrant they're located in
            // Craft if statement blocks according to position of p_0_4

            //     2  |  1
            //    ----.----
            //     3  |  4

            if (foot_x[leg_iterator] <= 0 && foot_y[leg_iterator] <= 0)
            {
                // We're in the lower left quadrant, quadrant 3
                theta_list[0 + 3 * leg_iterator] = PI - alpha_angle - beta_angle;
            }
            else if (foot_x[leg_iterator] <= 0 && foot_y[leg_iterator] > 0)
            {
                // We're in the upper left quadrant, quadrant 2
                theta_list[0 + 3 * leg_iterator] = alpha_angle + (PI - beta_angle);
            }
            else if (foot_x[leg_iterator] > 0 && foot_y[leg_iterator] > 0)
            {
                // We're in the upper right quadrant, quadrant 1
                theta_list[0 + 3 * leg_iterator] = alpha_angle + (PI - beta_angle);
            }
            else
            {
                // We're in the lower right quadrant, quadrant 4
                theta_list[0 + 3 * leg_iterator] = PI - alpha_angle - beta_angle;
            }



            Matrix<float, 3, 1> p_0_1;
            p_0_1 << -hip_length * cos( theta_list[0 + 3 * leg_iterator] ), -hip_length * sin( theta_list[0 + 3 * leg_iterator] ), 0;

            cout << "\np_0_1:\n" << p_0_1 << endl;

            Matrix<float, 3, 1> u_vec;
            //float p_to_u_factor = u_vec_magnitude / p_4_magnitude; -- this was for the previuus formulation of u_vec, which said that u_vec and p_0_4 point in
            //  the same direction. This is not the case, u_vec points in the direction of p_0_4 - p_0_1
            float p_to_u_factor = u_vec_magnitude / tau; 

            u_vec = p_to_u_factor * (p_0_4 - p_0_1);

            Matrix<float, 3, 1> v_vec;
            v_vec = v_vec_magnitude * u_vec.cross(p_0_1);

            // Creating p_0_3, in terms of u_vec and v_vec (which are in turn based on p_0_4) -- p_0_3 = p_0_1 + u_vec + v_vec;
            Matrix<float, 3, 1> p_0_3;
            p_0_3 = p_0_1 + u_vec + v_vec;
            float p_3_magnitude = p_0_3.norm();

            /*
            if (leg_iterator == 0)
            {
                cout << "p_0_1:\n"
                     << p_0_1 << "\nu_vec:\n"
                     << u_vec << "\nv_vec:\n"
                     << v_vec << endl;

                cout << "p_0_3:\n"
                     << p_0_3 << "\n p_3 - p_1 dotted with u_vec"
                     << (p_0_3-p_0_1).dot(u_vec) << "\nu_vec_magnitude * shoulder_length:\n"
                     << u_vec_magnitude * shoulder_length << endl;

                cout << "\natan2(v_vec, u_vec):\n"
                     << atan2(v_vec_magnitude, u_vec_magnitude) << endl;

                cout << "poop: " << sqrt(u_vec_magnitude * u_vec_magnitude + v_vec_magnitude * v_vec_magnitude) << endl;
            }
            */

            //theta_list[1 + 3 * leg_iterator] = acos( (p_0_3 - p_0_1).dot(-y_direction) / p_3_magnitude );

            theta_list[1 + 3 * leg_iterator] = acos( (float)(p_0_3 - p_0_1).dot(u_vec) / (shoulder_length * u_vec_magnitude) );



            theta_list[2 + 3 * leg_iterator] = M_PI - phi_angle;



        }


        cout << "I did IK!" << endl;
    }

    Matrix4d makeTransformMatrix(Matrix3d rotation, Vector3d translation)
    {
        Matrix4d transform;

        transform.setIdentity();
        transform.block<3,3>(0,0) = rotation;
        transform.block<3,1>(0,3) = translation;

        return transform;
    }

    Matrix<float, 4, 4> makeSkewSymmetricScrewMatrix(Matrix<float, 6, 1> screw_axis)
    {
        // precondition -- input should be a 6x1 vector, with the first 3 elements corresponding to angular velocities, and the last 3 corresponding to
        //                   linear velocities.
        // postcondition -- output should provide a 4x4 matrix that represents the skew-symmetric representation of screw axis parameter
        Matrix<float, 4, 4> skew_transform;

        skew_transform.row(0) << 0             , -screw_axis[2], screw_axis[1] , screw_axis[3];
        skew_transform.row(1) << screw_axis[2] , 0             , -screw_axis[0], screw_axis[4];
        skew_transform.row(2) << -screw_axis[1], screw_axis[0] , 0             , screw_axis[5];
        skew_transform.row(3) << 0             , 0             , 0             , 0            ;

        return skew_transform;
    }


    // *** Mutator functions *** This isn't super fleshed out at the moment

    // Mutator funciton?
    // not sure if necessary here
    void setRotation(Matrix3d &rotation, int axis, float theta_angle)
    {
        // (0, 0, 0)
        // (0, 0, 0)
        // (0, 0, 0)

        // needs switch case to handle different axis -- principal
        // change type of axis parameter to appropriate type

        rotation(0, 0) = 0;
        rotation(0, 1) = 0;
        rotation(0, 2) = 0;
        rotation(1, 0) = 0;
        rotation(1, 1) = 0;
        rotation(1, 2) = 0;
        rotation(2, 0) = 0;
        rotation(2, 1) = 0;
        rotation(2, 2) = 0;

    }

    // *** End of mutator functions ***



    // *** Accessor Functions *** Doesn't do anything right now

    // Gets current transformation matrix from the {0} frame to the {4} frame for a particular arm
    Matrix<float, 4, 4> getForwardKinematics()
    {
        // Input should be a string -- "front_right", "front_left", "rear_right", "rear_left"
        Matrix<float, 4, 4> output_transform;

        return output_transform;
    }

    // *** End of Accessor Functions ***






    // *** Print functions ***

    //   Goal is to make functions that print to the terminal.
    //   They should probably also return them so other functions can use them, but not sure yet
    void printThetaList()
    {
        // Prints out current theta_list -- the angles for each joint
        cout << "Current configuration is: " << theta_list << endl;
    }

    // *** End of Print functions ***




};






// Main function
// Goal is to test FK and IK -- create an instance of kinematics, and run the forward and inverse functions on it.
int main()
{
    
    // Creating DogKinematics object
    //DogKinematics doggo_1 = DogKinematics(1.0, 1.0, 0.1, 0.2, 0.2);
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
    doggo_1.theta_list << M_PI / 2, M_PI / 4, 3 * M_PI / 8, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f;

    // chassis to hip transforms
    doggo_1.transform_m_to_front_right_0 << 
        cos(PI/2)  , 0        , sin(PI/2)  , doggo_1.robot_length / 2 ,
        0          , 1        , 0          , 0                        ,
        -sin(PI/2) , 0        , cos(PI/2)  , doggo_1.robot_width / 2  ,
        0          , 0        , 0          , 1                        ;

    doggo_1.transform_m_to_front_left_0 <<  
        cos(-PI/2) , 0        , sin(-PI/2) , doggo_1.robot_length / 2 ,
        0          , 1        , 0          , 0                        ,
        -sin(-PI/2), 0        , cos(-PI/2) , -doggo_1.robot_width / 2 ,
        0          , 0        , 0          , 1                        ;

    doggo_1.transform_m_to_rear_right_0 <<  
        cos(PI/2)  , 0        , sin(PI/2)  , -doggo_1.robot_length / 2,
        0          , 1        , 0          , 0                        ,
        -sin(PI/2) , 0        , cos(PI/2)  , doggo_1.robot_width / 2  ,
        0          , 0        , 0          , 1                        ;

    doggo_1.transform_m_to_rear_left_0 <<   
        cos(-PI/2) , 0        , sin(-PI/2) , -doggo_1.robot_length / 2,
        0          , 1        , 0          , 0                        ,
        -sin(-PI/2), 0        , cos(-PI/2) , -doggo_1.robot_width / 2 ,
        0          , 0        , 0          , 1                        ;


    // Initilizing M, the home matrix
    doggo_1.home_mat <<
        0, 0, -1, -doggo_1.hip_length,
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

    cout << "Transform from front right hip {0} to front right foot {4} is:\n" << doggo_1.trans_front_right_0_4 << endl;
    //cout << "Transform from front left hip {0} to front left foot {4} is:\n" << doggo_1.trans_front_left_0_4 << endl;
    //cout << "Transform from rear right hip {0} to rear right foot {4} is:\n" << doggo_1.trans_rear_right_0_4 << endl;
    //cout << "Transform from rear left hip {0} to rear left foot {4} is:\n" << doggo_1.trans_rear_left_0_4 << endl;

    first_foot_transform = doggo_1.trans_front_right_0_4;




    //doggo_1.findNewFK(doggo_1.theta_list);

    //cout << "Done a different way, the transform from front right hip {0} to front right foot {4} is:\n" << doggo_1.trans_front_right_0_4 << endl;

    //cout << "Done a different way, the transform from front right hip {0} to front left foot {4} is:\n" << doggo_1.trans_front_left_0_4 << endl;

    //cout << "Done a different way, the transform from front rear hip {0} to front right foot {4} is:\n" << doggo_1.trans_rear_right_0_4 << endl;

    //cout << "Done a different way, the transform from front rear hip {0} to front left foot {4} is:\n" << doggo_1.trans_rear_left_0_4 << endl;
    



    doggo_1.calculateInverseKinematics(doggo_1.trans_front_right_0_4, doggo_1.trans_front_left_0_4, doggo_1.trans_rear_right_0_4, doggo_1.trans_rear_left_0_4);

    cout << "Current theta_list:\n" << doggo_1.theta_list << endl;

    second_theta_list = doggo_1.theta_list;

    //transform = doggo_1.makeTransformMatrix(rotation, translation);

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
    cout << "\nError between 1st and 2nd run through of FK:\n" << second_foot_transform - first_foot_transform << endl;

    cout << "\nError between initial theta_list and theta_list after IK\n" << second_theta_list - first_theta_list << endl;

    return 0;
}