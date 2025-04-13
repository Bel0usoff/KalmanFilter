#include <cmath>
#include <Eigen/Dense>
#include <random>
#include "extendedKalman.hpp"


ExtendedKalmanFilter :: ExtendedKalmanFilter(int numbers_of_points)
{
    for(int i = 0;i<numbers_of_points;i++)
    {
        state.push_back(Eigen :: VectorXd(5));
    }
}


std::vector <double> ExtendedKalmanFilter :: GenerateNoise(int samples,double mean, double stddev)
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<double> dist(mean, stddev);

    std::vector<double> noise(samples);
    for (auto& sample : noise) {
        sample = dist(gen);
    }
    return noise;
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


