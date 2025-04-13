#include <vector>
#include "extendedKalman.hpp"
#include <Eigen/Dense>
#include <iostream>

int main()
{

    ExtendedKalmanFilter EKF(10);
    EKF.InitFilter(0,0,0,0);
    std :: cout << EKF.GetState(0)<<'\n';
    return 0;
    
}
