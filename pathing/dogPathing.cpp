// using eigen -- https://gitlab.com/libeigen/eigen
// adding eigen -- use #include <Eigen/Dense> -- when compiling, provide path to eigen library -- eigen-3.4.0
// for me g++ -I eigen-3.4.0 file_name.cpp -o desired_compiled_file_name


// more references for eigen -- https://aleksandarhaber.com/starting-with-eigen-c-matrix-library/

#include "dogPathing.h"


#include <iostream>
#include <string>
#include <stdio.h> /* printf */
#include <math.h>  /* atan2 */
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <vector> // For storing multiple end effector configurations into an iterable vector

#include <unsupported/Eigen/MatrixFunctions>

using Eigen::Matrix;
using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using namespace Eigen;

using namespace std;


// Currently this class makes trajectories that are X by 12, and so each individual row is a 1 by 12 matrix
// This is different than how the other class has its joint matrix -- it uses a 12 by 1 matrix
// I like the appearance of X rows of 1 by 12s, but I should be consistent with the rest of my code

/*
class DogPathing
{
    public:
        DogPathing();
        Matrix<float, Dynamic, 12> createFullJointTrajectory(Matrix<float, 12, 1> theta_list_start, Matrix<float, 12, 1> theta_list_end, float total_motion_time, int number_descrete_points, int time_scaling_method);
        Matrix<float, Dynamic, 3> createLegJointTrajectory(Matrix<float, 3, 1> theta_list_start, Matrix<float, 3, 1> theta_list_end, float total_motion_time, int number_descrete_points, int time_scaling_method);
        vector<Matrix<float, 4, 4> > createScrewTrajectory(Matrix<float, 4, 4> start_configuration, Matrix<float, 4, 4> end_configuration, float total_motion_time, int number_descrete_points, int time_scaling_method);
        vector<Matrix<float, 4, 4> > createCartesianTrajectory(Matrix<float, 4, 4> start_configuration, Matrix<float, 4, 4> end_configuration, float total_motion_time, int number_descrete_points, int time_scaling_method);

        float cubicTimeScaling(float total_motion_time, float current_time);
        float quinticTimeScaling(float total_motion_time, float current_time);

        // These functions might need to exist outside of this class
        Matrix<float, 3, 3> extractRotation(Matrix<float, 4, 4> input_transform);
        Matrix<float, 3, 1> extractPosition(Matrix<float, 4, 4> input_transform);
        Matrix<float, 4, 4> matrixLogarithmOfTransform(Matrix<float, 4, 4> input_transform);
        Matrix<float, 3, 3> makeSkewSymmetricMatrix33(Matrix<float, 3, 1> input_vector);
        Matrix<float, 4, 4> makeSkewSymmetricMatrix44(Matrix<float, 6, 1> screw_axis);

    private:

};

*/

DogPathing::DogPathing()
{
 
}

Matrix<float, Dynamic, 12> DogPathing::createFullJointTrajectory(Matrix<float, 12, 1> theta_list_start, Matrix<float, 12, 1> theta_list_end, float total_motion_time, int number_descrete_points, int time_scaling_method)
{
    // Allocates space for a matrix with rows = number_discrete_points, and collumns = 12
    // Each row should have 12 elements corresponding to the 12 joint angles of the robot, from theta_list_start to theta_list_end
    MatrixXf joint_traj(number_descrete_points, 12);

    float time_gap = total_motion_time / (number_descrete_points - 1);

    float path_parameter;

    // Iterate through all points in desired trajectory, indicated by number_descrete_points
    for (int i = 0; i < number_descrete_points; i++)
    {
        switch (time_scaling_method)
        {
        case 3:             // Cubic Time Scaling
            path_parameter = cubicTimeScaling(total_motion_time, time_gap * i);
            break;
        case 5:             // Quintic Time Scaling
            path_parameter = quinticTimeScaling(total_motion_time, time_gap * i);
            break;

        default:
            cout << "Not a supported time scaling" << endl;
            break;
        }

        //   traj(:, i) = thetastart + s * (thetaend - thetastart);
        //
        for (int j = 0; j < theta_list_start.rows(); j++)
        {
            joint_traj(i, j) = theta_list_start[j] + path_parameter * (theta_list_end[j] - theta_list_start[j]);
        }

    }

    return joint_traj;

}



Matrix<float, Dynamic, 3> DogPathing::createLegJointTrajectory(Matrix<float, 3, 1> theta_list_start, Matrix<float, 3, 1> theta_list_end, float total_motion_time, int number_descrete_points, int time_scaling_method)
{

    // Allocates space for a matrix with rows = number_discrete_points, and collumns = 3
    // Each row should have 3 elements corresponding to the 3 joint angles of one of the robot's arms, from theta_list_start to theta_list_end
    MatrixXf joint_traj(number_descrete_points, 3);

    float time_gap = total_motion_time / (number_descrete_points - 1);

    float path_parameter;

    // Iterate through all points in desired trajectory, indicated by number_descrete_points
    for (int i = 0; i < number_descrete_points; i++)
    {
        switch (time_scaling_method)
        {
        case 3: // Cubic Time Scaling
            path_parameter = cubicTimeScaling(total_motion_time, time_gap * i);
            break;
        case 5: // Quintic Time Scaling
            path_parameter = quinticTimeScaling(total_motion_time, time_gap * i);
            break;

        default:
            cout << "Not a supported time scaling" << endl;
            break;
        }

        //   traj(:, i) = thetastart + s * (thetaend - thetastart);
        //
        for (int j = 0; j < theta_list_start.rows(); j++)
        {
            joint_traj(i, j) = theta_list_start[j] + path_parameter * (theta_list_end[j] - theta_list_start[j]);
        }
    }

    return joint_traj;
}



vector<Matrix<float, 4, 4> > DogPathing::createScrewTrajectory(Matrix<float, 4, 4> start_configuration, Matrix<float, 4, 4> end_configuration, float total_motion_time, int number_descrete_points, int time_scaling_method)
{
    // Trajectory vector -- this is a vector of 4x4 matrices that represent the end effector configurations from the start_configuration
    //    to the end_configuration
    vector<Matrix<float, 4, 4> > configuration_screw_trajectory;

    if (number_descrete_points <= 0)
    {
        cout << "Not enough points in trajectory, please choose a number of points greater than 0" << endl;
        return configuration_screw_trajectory;
    }

    float time_gap = total_motion_time / (number_descrete_points - 1);

    float path_parameter;

    Matrix<float, 4, 4> current_configuration;

    // Iterate through all points in desired trajectory, indicated by number_descrete_points
    for (int i = 0; i < number_descrete_points; i++)
    {
        switch (time_scaling_method)
        {
        case 3: // Cubic Time Scaling
            path_parameter = cubicTimeScaling(total_motion_time, time_gap * i);
            break;
        case 5: // Quintic Time Scaling
            path_parameter = quinticTimeScaling(total_motion_time, time_gap * i);
            break;

        default:
            cout << "Not a supported time scaling" << endl;
            break;
        }

        current_configuration = start_configuration * ((start_configuration.inverse() * end_configuration).log() * path_parameter).exp();
        configuration_screw_trajectory.push_back(current_configuration);
    }

    return configuration_screw_trajectory;
}

vector<Matrix<float, 4, 4> > DogPathing::createCartesianTrajectory(Matrix<float, 4, 4> start_configuration, Matrix<float, 4, 4> end_configuration, float total_motion_time, int number_descrete_points, int time_scaling_method)
{
    vector<Matrix<float, 4, 4> > configuration_cartesian_trajectory;

    return configuration_cartesian_trajectory;
}


/// Functions that might need to be in a different class /// 

Matrix<float, 4, 4> DogPathing::matrixLogarithmOfTransform(Matrix<float, 4, 4> input_transform)
{
    // this is technically conflating matrix logarithm in 6 coor with 3 coor but it should be fine

    // Defining minimum error tolerance for seeing if a matrix is close enough to identity
    float eps = 0.001;

    Matrix<float, 3, 3> rotation;
    Matrix<float, 3, 1> position;
    float theta;
    Matrix<float, 3, 1> angular_velocity;
    Matrix<float, 3, 3> skew_angular_velocity;
    Matrix<float, 3, 1> linear_velocity;
    Matrix<float, 3, 3> identity;
    identity.setIdentity();

    Matrix<float, 6, 1> screwVector;

    rotation = extractRotation(input_transform);
    position = extractPosition(input_transform);

    if (rotation.isIdentity(eps))
    {
        angular_velocity << 0, 0, 0;
        linear_velocity = position / position.norm();
        theta = position.norm();

    }
    else if (rotation.trace() == -1)
    {
        theta = M_PI;
        cout << "I'm confused about what angular velocity should be here" << endl;
    }
    else
    {
        theta = acos(0.5 * (rotation.trace() - 1));
        skew_angular_velocity = (1 / (2 * sin(theta) ) ) * (rotation - rotation.transpose());
        linear_velocity = ( (1 / theta) * identity 
                        - 0.5 * skew_angular_velocity
                        + ( (1 / theta) - 0.5 * ( cos(theta/2.0) / sin(theta/2.0) ) ) * skew_angular_velocity * skew_angular_velocity ) * position;
    }

    screwVector.block<3, 1>(0, 0) = angular_velocity;
    screwVector.block<3, 1>(3, 0) = linear_velocity;

    // Still need to add a * theta to here //

    return makeSkewSymmetricMatrix44(screwVector) * theta;
}

Matrix<float, 3, 3> DogPathing::extractRotation(Matrix<float, 4, 4> input_transform)
{
    Matrix<float, 3, 3> rotation;
    rotation = input_transform.block<3, 3>(0, 0);


    return rotation;
}

Matrix<float, 3, 1> DogPathing::extractPosition(Matrix<float, 4, 4> input_transform)
{
    Matrix<float, 3, 1> position;
    position = input_transform.block<3, 1>(0, 3);
    return position;
}

Matrix<float, 3, 3> DogPathing::makeSkewSymmetricMatrix33(Matrix<float, 3, 1> input_vector)
{
    // precondition -- input should be a 6x1 vector, with the first 3 elements corresponding to angular velocities, and the last 3 corresponding to
    //                   linear velocities.
    // postcondition -- output should provide a 4x4 matrix that represents the skew-symmetric representation of screw axis parameter
    Matrix<float, 3, 3> skew_vector;

    skew_vector.row(0) << 0, -input_vector[2], input_vector[1];
    skew_vector.row(1) << input_vector[2], 0, -input_vector[0];
    skew_vector.row(2) << -input_vector[1], input_vector[0], 0;

    return skew_vector;
}

Matrix<float, 4, 4> DogPathing::makeSkewSymmetricMatrix44(Matrix<float, 6, 1> screw_axis)
{
    // precondition -- input should be a 6x1 vector, with the first 3 elements corresponding to angular velocities, and the last 3 corresponding to
    //                   linear velocities.
    // postcondition -- output should provide a 4x4 matrix that represents the skew-symmetric representation of screw axis parameter
    Matrix<float, 4, 4> skew_transform;

    skew_transform.row(0) << 0, -screw_axis[2], screw_axis[1], screw_axis[3];
    skew_transform.row(1) << screw_axis[2], 0, -screw_axis[0], screw_axis[4];
    skew_transform.row(2) << -screw_axis[1], screw_axis[0], 0, screw_axis[5];
    skew_transform.row(3) << 0, 0, 0, 0;

    return skew_transform;
}

/// End of Functions that might need to be in a different class ///





//// Time Scaling functions ////
float DogPathing::cubicTimeScaling(float total_motion_time, float current_time)
{
    float path_parameter;

    path_parameter = 3 * pow((current_time / total_motion_time), 2.0) - 2 * pow((current_time / total_motion_time), 3.0);

    return path_parameter;
}

float DogPathing::quinticTimeScaling(float total_motion_time, float current_time)
{
    float path_parameter;

    path_parameter = 10 * pow((current_time / total_motion_time), 3.0) - 15 * pow((current_time / total_motion_time), 4.0)
                                + 6 * pow((current_time / total_motion_time), 5.0);

    return path_parameter;
}
//// End of Time Scaling Functions ////

/*
int main()
{
    DogPathing dummy_path;

    int scaling_type = 5;
    float final_time = 10.0;
    int traj_point_num = 2;

    /////////////

    Matrix<float, 4, 4> myStart;
    myStart.row(0) << 1, 0, 0, 0;
    myStart.row(1) << 0, 1, 0, 0;
    myStart.row(2) << 0, 0, 1, 0;
    myStart.row(3) << 0, 0, 0, 1;

    Matrix<float, 4, 4> myEnd;
    myEnd.row(0) << 1, 0, 0, 0;
    myEnd.row(1) << 0, 1, 0, 0;
    myEnd.row(2) << 0, 0, 1, 6;
    myEnd.row(3) << 0, 0, 0, 1;

    vector<Matrix<float, 4, 4> > dummy_traj_third;
    dummy_traj_third = dummy_path.createScrewTrajectory(myStart, myEnd, final_time, traj_point_num, scaling_type);

    for (Matrix<float, 4, 4> configuration : dummy_traj_third)
    {
        cout << "Current Configuration:\n" << configuration << endl;
    }

    return 0;
}
*/