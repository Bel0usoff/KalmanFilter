#include <cmath>
#include <Eigen/Dense>
#include <random>
#include "extendedKalman.hpp"


const double kMean = 0.0;
const double kStdDev = 0.01;


ExtendedKalmanFilter :: ExtendedKalmanFilter(int numbers_of_points)
{
    for(int i = 0;i<numbers_of_points;i++)
    {
        state.push_back(Eigen :: VectorXd(5));
    }
}


double ExtendedKalmanFilter :: GenerateNoise(double mean, double stddev)
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<double> dist(mean, stddev);
    return dist(gen);
}

void ExtendedKalmanFilter :: InitFilter(double Ox,double Oy,double velocity,double angular_velocity)
{
    state[0](0) = Ox; //координата по Х
    state[0](1) = Oy; //координата по У
    state[0](3) = velocity; //путевая скорость
    state[0](4) = angular_velocity; //угловая скорость   
}

Eigen :: VectorXd ExtendedKalmanFilter :: GetState(int index_state)
{
    return state[index_state];
}

void ExtendedKalmanFilter :: PredictStep(Eigen :: VectorXd& cur_state,Eigen :: VectorXd& prev_state)
{
    //расчет текущего вектора состояния через прошлый
    cur_state(0) = prev_state(0) + prev_state(3)*dt*cos(prev_state(2));//Ox
    cur_state(1) = prev_state(1) + prev_state(3)*dt*cos(prev_state(2));//Oy
    cur_state(2) = prev_state(2) + prev_state(4)*dt;//направление
    cur_state(3) = prev_state(3) + GenerateNoise(kMean,kStdDev);//путевая скорость
    cur_state(4) = prev_state(4) + GenerateNoise(kMean,kStdDev);//угловая скорость
}

