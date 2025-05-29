#include "inc.hpp"

int main()
{

    ExtendedKalmanFilter EKF(600);// потом придумать как не хардкодить число эелементов
   
    EKF.LoadGpsFromCSV("/home/konstantin/kalman_filter/program/trajectory.csv");
    EKF.RunEKF();
    for(int i = 0; i< 600; i++)
    {
        std :: cout << EKF.GetState(i) << std :: endl;
        std :: cout << std :: endl;
    }
    return 0;
    
}
