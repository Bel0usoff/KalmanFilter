#pragma once

#include "inc.hpp"



struct GpsData
{
    double timestamp;
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
        Eigen :: MatrixXd Q;    // Шум модели
     
        double const dt = 0.1;          // Шаг времени
    
        double GenerateNoise(double mean, double stddev);

    public:

        void PrintGPS();    

        int GetPointsCount();

        int GetSize();

        void SetState(int index,double x,double y,double v,double omega);

        //void IncreaseDt();

        void InitFilter(double Ox,double Oy,double velocity,double angular_velocity);

        void PredictStep(Eigen::VectorXd& cur_state,
            const Eigen::VectorXd& prev_state);

        void UpdateStep (Eigen::VectorXd& state_vec,
            const Eigen::Vector2d& z);

        Eigen :: VectorXd& GetState(int index_state);

        ExtendedKalmanFilter(int number_of_points);

        void LoadGpsFromCSV(const std :: string& filename);

        void RunEKF();

        void ExportGpsToCSV (const std :: string& FileNameOut);
};