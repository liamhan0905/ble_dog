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

class DogDynamics
{
    public:
        DogDynamics(); // Constructor
    private:
        
};

int main()
{
    return 0;
}