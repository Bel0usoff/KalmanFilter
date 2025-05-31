#include <iostream>
#include <fstream>
#include <vector>
#include <random>
#include <cmath>
#include <iomanip>
#include <string>
#include <cstdlib>
#include <chrono>

using namespace std;

struct GPSPoint {
    double latitude;    // Широта в градусах
    double longitude;   // Долгота в градусах
    double timestamp;   // Временная метка (секунды)
};

struct SimulationParams {
    double startLat = 55.752220;  // Стартовая широта (Москва)
    double startLon = 37.615560;  // Стартовая долгота
    double speed = 20.0;          // Средняя скорость (м/с)
    double speedVar = 0;        // Разброс скорости
    double noiseLevel = 2.0;      // Уровень шума (м)
    double duration = 300.0;      // Длительность полета (сек)
    double interval = 1.0;        // Интервал между точками (сек)
    string outputFile = "trajectory.csv";
};

class GPSGenerator {
    mt19937 generator;
    normal_distribution<double> noiseDist;
    uniform_real_distribution<double> speedVarDist;
    uniform_real_distribution<double> courseChangeDist;
    SimulationParams params;
    GPSPoint currentPoint;
    double course; // Текущий курс (радианы)

public:
   GPSGenerator(const SimulationParams& p)

: params(p),
generator(static_cast<unsigned long>(
chrono::system_clock::now().time_since_epoch().count())),
noiseDist(0.0, p.noiseLevel / 111320.0),
courseChangeDist(-0.1, 0.1),
currentPoint{p.startLat, p.startLon, 0.0},


course(0.0) {} 

    GPSPoint generateNextPoint() {
        currentPoint.timestamp += params.interval;
        
        double currentSpeed = params.speed + speedVarDist(generator);
        course += courseChangeDist(generator);
        
        double distance = currentSpeed * params.interval;
        double dx = distance * sin(course);
        double dy = distance * cos(course);
        
        currentPoint.latitude += dx / 111320.0;
        currentPoint.longitude += dy / (111320.0 * cos(currentPoint.latitude * M_PI / 180.0));
        
        GPSPoint noisyPoint = currentPoint;
        noisyPoint.latitude += noiseDist(generator);
        noisyPoint.longitude += noiseDist(generator);
        
        return noisyPoint;
    }

    vector<GPSPoint> generateTrajectory() {
        vector<GPSPoint> trajectory;
        int numPoints = static_cast<int>(params.duration / params.interval);
        trajectory.reserve(numPoints);
        
        for (int i = 0; i < numPoints; ++i) {
            trajectory.push_back(generateNextPoint());
        }
        
        return trajectory;
    }
};

void saveToCSV(const vector<GPSPoint>& trajectory, const string& filename) {
    ofstream outFile(filename);
    if (!outFile) {
        cerr << "Error opening file: " << filename << endl;
        exit(1);
    }

    outFile << "timestamp,latitude,longitude\n";
    outFile << fixed << setprecision(8);
    
    for (const auto& point : trajectory) {
        outFile << point.timestamp << ","
                << point.latitude << ","
                << point.longitude << "\n";
    }
    
    cout << "Generated " << trajectory.size() << " GPS points to " << filename << endl;
}

void printHelp() {
    cout << "UAV GPS Data Generator\n"
         << "Usage: gps_generator [options]\n"
         << "Options:\n"
         << "  --lat <value>     Starting latitude (default: 55.752220)\n"
         << "  --lon <value>     Starting longitude (default: 37.615560)\n"
         << "  --speed <value>   Average speed m/s (default: 20.0)\n"
         << "  --noise <value>   GPS noise in meters (default: 2.0)\n"
         << "  --time <value>    Flight duration seconds (default: 300)\n"
         << "  --interval <value> Time between points (default: 1.0)\n"
         << "  --output <file>   Output CSV file (default: trajectory.csv)\n"
         << "  --help            Show this help\n";
}

SimulationParams parseArgs(int argc, char* argv[]) {
    SimulationParams params;
    
    for (int i = 1; i < argc; ++i) {
        string arg = argv[i];
        if (arg == "--lat" && i+1 < argc) {
            params.startLat = stod(argv[++i]);
        } else if (arg == "--lon" && i+1 < argc) {
            params.startLon = stod(argv[++i]);
        } else if (arg == "--speed" && i+1 < argc) {
            params.speed = stod(argv[++i]);
        } else if (arg == "--noise" && i+1 < argc) {
            params.noiseLevel = stod(argv[++i]);
        } else if (arg == "--time" && i+1 < argc) {
            params.duration = stod(argv[++i]);
        } else if (arg == "--interval" && i+1 < argc) {
            params.interval = stod(argv[++i]);
        } else if (arg == "--output" && i+1 < argc) {
            params.outputFile = argv[++i];
        } else if (arg == "--help") {
            printHelp();
            exit(0);
        }
    }
    
    return params;
}

int main(int argc, char* argv[]) {
    SimulationParams params = parseArgs(argc, argv);
    GPSGenerator generator(params);
    auto trajectory = generator.generateTrajectory();
    saveToCSV(trajectory, params.outputFile);
    return 0;
}