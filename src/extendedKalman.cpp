#include "inc.hpp"


const double kMean = 0.0;
const double kStdDev = 0.001;

//void ExtendedKalmanFilter :: IncreaseDt() {dt += 0.5;} 
int ExtendedKalmanFilter :: GetSize()
{
    return this->state.size();
}

int ExtendedKalmanFilter :: GetPointsCount()
{
    return this->GPS_values.size();
}

void ExtendedKalmanFilter :: SetState(int index,double Ox,double Oy,double velocity,double angular_velocity)
{
    state[index](0) = Ox; //координата по Х
    state[index](1) = Oy; //координата по У
    state[index](3) = velocity; //путевая скорость
    state[index](4) = angular_velocity; //угловая скорость  
}
ExtendedKalmanFilter :: ExtendedKalmanFilter(int number_of_points)
{
    for(int i = 0;i<number_of_points;i++)
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

Eigen :: VectorXd& ExtendedKalmanFilter :: GetState(int index_state)
{
    return state[index_state];
}

void ExtendedKalmanFilter :: PredictStep(Eigen :: VectorXd& cur_state,Eigen :: VectorXd& prev_state)
{
    //расчет текущего вектора состояния через прошлый
    cur_state(0) = prev_state(0) + prev_state(3)*dt*cos(prev_state(2));//Ox
    cur_state(1) = prev_state(1) + prev_state(3)*dt*cos(prev_state(2));//Oy
    cur_state(2) = prev_state(2) + prev_state(4)*dt;//направление
    cur_state(3) = prev_state(3);// + GenerateNoise(kMean,kStdDev);//путевая скорость
    cur_state(4) = prev_state(4) ;//+ GenerateNoise(kMean,kStdDev);//угловая скорость
    
    
    Eigen :: MatrixXd jacobi(5,5);
    jacobi <<   1, 0, 0, 0,  dt*cos(prev_state(2)),
                0, 1, 0, 0,  dt*sin(prev_state(2)),
                0, 0, 1, dt, 0,
                0, 0, 0, 1,  0,
                0, 0, 0, 0,  0;
                
    cur_state = jacobi * cur_state;
}
 void  ExtendedKalmanFilter :: LoadGpsFromCSV(const std :: string& filename)
{
    std::ifstream file(filename);
    std::string line;
    int i = 0;
    // Пропуск заголовка
    std::getline(file, line);
    
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        GpsData data;
        char comma;
        ss >> data.timestamp >> comma >> data.lat >> comma >> data.lon;
        GPS_values.push_back(data);
        
    }
}

