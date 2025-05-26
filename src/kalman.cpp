#include "inc.hpp"

int main()
{

    ExtendedKalmanFilter EKF(600);// потом придумать как не хардкодить число эелементов
    EKF.InitFilter(0,0,3,0);
    //EKF.SetState(1,1,1,3,0);
    //EKF.SetState(2,2,2,3,0);

    EKF.LoadGpsFromCSV("/home/konstantin/kalman_filter/program/trajectory.csv");
    for(int i = 0;i<EKF.GetPointsCount();i++)
    {
        
    }
    std :: cout << EKF.GetState(0)<<'\n';
    std :: cout << std :: endl;
    for(int i = 1;i<10;i++)
    {
        EKF.PredictStep(EKF.GetState(i), EKF.GetState(i-1));
        std :: cout << EKF.GetState(i)<<'\n';
        std :: cout << std :: endl;
    }
    return 0;
    
}
