#pragma once

#include "inc.hpp"



struct GpsData
{
    int timestamp;
    double lat,lon; //широта,долгота
};

class ExtendedKalmanFilter
{
    private:
        /* 
        x,y - координаты GPS ,
        η - направление
        ω - угловая скорость
        v - путевая скорость
        */

        std :: vector <GpsData> GPS_values;
        std :: vector <Eigen :: VectorXd> state; // вектор состояния [x, y, η, ω, v]
        Eigen :: MatrixXd P;     // Ковариационная матрица
        Eigen :: MatrixXd H;     // Матрица измерений
        Eigen :: MatrixXd R;     // Шум измерений
        double const dt = 0.1;          // Шаг времени
        double Q_omega;     // Дисперсия шума ω
        double Q_v;         // Дисперсия шума v


        double GenerateNoise(double mean, double stddev);

    public:

        int GetSize();

        void SetState(int index,double x,double y,double v,double omega);

        //void IncreaseDt();

        void InitFilter(double Ox,double Oy,double velocity,double angular_velocity);

        void PredictStep(Eigen :: VectorXd& cur_state,Eigen :: VectorXd& prev_state);

        Eigen :: VectorXd& GetState(int index_state);

        ExtendedKalmanFilter(int number_of_points);

        void LoadGpsFromCSV(const std :: string& filename);
};