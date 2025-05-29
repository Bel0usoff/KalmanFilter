#include "inc.hpp"


int main()
{

    ExtendedKalmanFilter EKF(600);// потом придумать как не хардкодить число эелементов
   
    EKF.LoadGpsFromCSV("/home/konstantin/kalman_filter/program/trajectory.csv");
    EKF.RunEKF();
    /* for(int i = 0; i< 600; i++)
    {
        std :: cout << EKF.GetState(i) << std :: endl;
        std :: cout << std :: endl;
    } */

    EKF.ExportGpsToCSV("/home/konstantin/kalman_filter/program/trajectory_filtered.csv");
    plot_gps_trajectory("/home/konstantin/kalman_filter/program/trajectory.csv","/home/konstantin/kalman_filter/program/source");
    plot_gps_trajectory("/home/konstantin/kalman_filter/program/trajectory_filtered.csv","/home/konstantin/kalman_filter/program/filter");


    createOverlayPlot("/home/konstantin/kalman_filter/program/trajectory.csv","/home/konstantin/kalman_filter/program/trajectory_filtered.csv");

    //showInteractivePlot("/home/konstantin/kalman_filter/program/trajectory.csv");
    //showInteractivePlot("/home/konstantin/kalman_filter/program/trajectory_filtered.csv");
    return 0;
    
}
