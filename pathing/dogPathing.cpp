// using eigen -- https://gitlab.com/libeigen/eigen
// adding eigen -- use #include <Eigen/Dense> -- when compiling, provide path to eigen library -- eigen-3.4.0
// for me g++ -I eigen-3.4.0 file_name.cpp -o desired_compiled_file_name


// more references for eigen -- https://aleksandarhaber.com/starting-with-eigen-c-matrix-library/

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

// Currently this class makes trajectories that are X by 12, and so each individual row is a 1 by 12 matrix
// This is different than how the other class has its joint matrix -- it uses a 12 by 1 matrix
// I like the appearance of X rows of 1 by 12s, but I should be consistent with the rest of my code
class DogPathing
{
    public:
        DogPathing();
        Matrix<float, Dynamic, 12> createJointTrajectory(Matrix<float, 12, 1> theta_list_start, Matrix<float, 12, 1> theta_list_end, float total_motion_time, int number_descrete_points, int time_scaling_method);
        void createScrewTrajectory();
        void createCartesianTrajectory();

        float cubicTimeScaling(float total_motion_time, float current_time);
        float quinticTimeScaling(float total_motion_time, float current_time);

    private:

};

DogPathing::DogPathing()
{
 
}

Matrix<float, Dynamic, 12> DogPathing::createJointTrajectory(Matrix<float, 12, 1> theta_list_start, Matrix<float, 12, 1> theta_list_end, float total_motion_time, int number_descrete_points, int time_scaling_method)
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
            //
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

void DogPathing::createScrewTrajectory()
{

}

void DogPathing::createCartesianTrajectory()
{

}

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

int main()
{
    DogPathing dummy_path;
    Matrix<float, Dynamic, 12> dummy_traj;
    
    Matrix<float, 12, 1> current_joint;
    Matrix<float, 12, 1> target_joint;

    int scaling_type = 5;
    float final_time = 5.0;
    int traj_point_num = 10;

    current_joint << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    target_joint << 1, 0, 0, 0.5, 0, 0, 0, 0, 0, 0, 0, 0;

    dummy_traj = dummy_path.createJointTrajectory(current_joint, target_joint, final_time, traj_point_num, scaling_type);

    cout << "Traj:\n" << dummy_traj << endl;

    return 0;
}