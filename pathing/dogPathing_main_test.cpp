
#include "dogPathing.h"

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