#include "inc.hpp"

const double kMean = 0.0;
const double kStdDev = 0.001;

ExtendedKalmanFilter::ExtendedKalmanFilter(int number_of_points)
{
    state.reserve(number_of_points);
    
    P = Eigen::MatrixXd::Identity(5, 5) * 2;  // начальная ковариация
    Q = Eigen::MatrixXd::Identity(2, 2) * 0.05; // шум управления (2×2!)
    R = Eigen::MatrixXd::Identity(2, 2) * 0.01; // шум измерений (GPS)

    
    H = Eigen::MatrixXd(2, 5); // матрица наблюдения
    H << 1, 0, 0, 0, 0,
         0, 1, 0, 0, 0;
}



void ExtendedKalmanFilter::InitFilter(double Ox,double Oy,double velocity,double angular_velocity)
{
    Eigen::VectorXd init_state(5);
    init_state << Ox, Oy, 0.0, velocity, angular_velocity;

    state.clear();
    state.push_back(init_state);
}

void ExtendedKalmanFilter::PredictStep(Eigen::VectorXd& cur_state,
                                       const Eigen::VectorXd& prev_state)
{
    double x = prev_state(0);
    double y = prev_state(1);
    double theta = prev_state(2);
    double v = prev_state(3);
    double omega = prev_state(4);

    
    cur_state(0) = x + v * dt * cos(theta);
    cur_state(1) = y + v * dt * sin(theta);
    cur_state(2) = theta + omega * dt;
    cur_state(3) = v;///GenerateNoise(kMean, kStdDev);
    cur_state(4) = omega;// + GenerateNoise(kMean, kStdDev);

    // Якобиан
    Eigen::MatrixXd F(5, 5);
    F << 1, 0, -v * dt * sin(theta), dt * cos(theta), 0,
         0, 1,  v * dt * cos(theta), dt * sin(theta), 0,
         0, 0, 1,                    0,               dt,
         0, 0, 0,                    1,               0,
         0, 0, 0,                    0,               1;

    // Матрица влияния шума управления
    Eigen::MatrixXd U(5, 2);
    U << 0,        0,
         0,        0,
         0,        0,
         sqrt(dt), 0,
         0, sqrt(dt);

    // Обновляем ковариацию
    P = F * P * F.transpose() + U * Q * U.transpose();
}

void ExtendedKalmanFilter::UpdateStep(Eigen::VectorXd& state_vec,
                                      const Eigen::Vector2d& z)
{
    Eigen::Vector2d z_pred;
    z_pred << state_vec(0), state_vec(1);

    Eigen::Vector2d y = z - z_pred;

    Eigen::MatrixXd S = H * P * H.transpose() + R;
    Eigen::MatrixXd K = P * H.transpose() * S.inverse();

    state_vec = state_vec + K * y;
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(state_vec.size(), state_vec.size());
    P = (I - K * H) * P;
}

void ExtendedKalmanFilter::RunEKF()
{
    if (GPS_values.empty()) {
        std::cerr << "Нет данных GPS\n";
        return;
    }


     InitFilter(GPS_values[0].lat, GPS_values[0].lon, 0.0, 0.0);
   
    Eigen::VectorXd estimated_state = state[0];


    int N = GPS_values.size();
    for (int i = 1; i < N; ++i)
    {
        Eigen::VectorXd predicted_state = estimated_state;
        PredictStep(predicted_state, estimated_state);

        Eigen::Vector2d Z;
        Z << GPS_values[i].lat, GPS_values[i].lon;

        UpdateStep(predicted_state, Z);

        estimated_state = predicted_state;
        state.push_back(estimated_state);
    }

    std::cout << "Фильтрация завершена. Состояний: " << state.size() << std::endl;
}

void ExtendedKalmanFilter::LoadGpsFromCSV(const std::string& filename)
{
    std::ifstream file(filename);
    std::string line;
    std::getline(file, line); // пропуск заголовка

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        GpsData data;
        char comma;
        ss >> data.timestamp >> comma >> data.lat >> comma >> data.lon;
        GPS_values.push_back(data);
    }
}

void ExtendedKalmanFilter :: ExportGpsToCSV (const std :: string& FileNameOut)
{
    std :: ofstream file(FileNameOut);
    if (!file.is_open()) {
        std::cerr << "Ошибка при открытии файла!" << std::endl;
        return;
    }

    file << "timestamp,latitude,longitude"<<"\n";
    for (int i = 0; i < this->GetSize();i++) 
    {
        file << i+1 << "," << state[i](0) << "," << state[i](1)<<"\n";
    }
    
}

void ExtendedKalmanFilter::PrintGPS()
{
    for (const auto& data : GPS_values)
        std::cout << data.timestamp << '\t' << data.lat << '\t' << data.lon << '\n';
}

double ExtendedKalmanFilter::GenerateNoise(double mean, double stddev)
{
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::normal_distribution<double> dist(mean, stddev);
    return dist(gen);
}

int ExtendedKalmanFilter::GetSize() { return state.size(); }

Eigen::VectorXd& ExtendedKalmanFilter::GetState(int i) { return state[i]; }