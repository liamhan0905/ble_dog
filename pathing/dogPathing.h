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