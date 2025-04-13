#pragma once

#include <cmath>
#include <Eigen/Dense>
#include <random>
#include <vector>

class ExtendedKalmanFilter
{
    private:
        /* 
        x,y - координаты GPS ,
        η - направление
        ω - угловая скорость
        v - путевая скорость
        */
        std :: vector <double> GPS_values;
        std :: vector <Eigen :: VectorXd> state; // вектор состояния [x, y, η, ω, v]
        Eigen :: MatrixXd P;     // Ковариационная матрица
        Eigen :: MatrixXd H;     // Матрица измерений
        Eigen :: MatrixXd R;     // Шум измерений
        double dt;          // Шаг времени
        double Q_omega;     // Дисперсия шума ω
        double Q_v;         // Дисперсия шума v


        double GenerateNoise(double mean, double stddev);

    public:

        void InitFilter(double Ox,double Oy,double velocity,double angular_velocity);

        void PredictStep(Eigen :: VectorXd& cur_state,Eigen :: VectorXd& prev_state);

        Eigen :: VectorXd GetState(int index_state);

        ExtendedKalmanFilter(int numbers_of_points);
};