#include "inc.hpp"

int main()
{

    ExtendedKalmanFilter EKF(600);// потом придумать как не хардкодить число эелементов
    EKF.InitFilter(55.7522,37.615,0,20);
    EKF.SetState(1,55.75224,37.616,20,0);
   // std :: cout << EKF.GetState(0)<<std::endl;
    std :: cout << std :: endl;
    EKF.LoadGpsFromCSV("/home/konstantin/kalman_filter/program/trajectory.csv");
   // EKF.PrintGPS();
   
    for(int i = 1;i<600;i++)
    {
        EKF.PredictStep(EKF.GetState(i), EKF.GetState(i-1));
        std :: cout << EKF.GetState(i)<<std::endl;
        std :: cout << std :: endl;
    }
    return 0;
    
}
