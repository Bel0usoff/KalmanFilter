cmake_minimum_required(VERSION 3.10)
project(ExtendedKalmanFilter)

# Установка стандарта C++ и включение Eigen
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Поиск Eigen3
find_package(Eigen3 REQUIRED)

# Включение заголовков
include_directories(
    include
    ${EIGEN3_INCLUDE_DIRS}
)

# Сборка исполняемого файла
add_executable(ExtendedKalmanFilter
    src/extendedKalman.cpp
    src/kalman.cpp
    src/visualization.cpp
)

# Линковка с Eigen (если требуется)
target_link_libraries(ExtendedKalmanFilter Eigen3::Eigen)