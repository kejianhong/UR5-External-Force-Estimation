cmake_minimum_required(VERSION 3.15)
project(UR5SimulationForceEstimation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE "Debug")
set(EXECUTABLE_OUTPUT_PATH "${CMAKE_SOURCE_DIR}/bin")

find_package(Boost REQUIRED COMPONENTS program_options iostreams system filesystem)
find_package(Eigen3 REQUIRED)

add_library(gnuplot_iostream INTERFACE)
target_include_directories(gnuplot_iostream INTERFACE "${CMAKE_SOURCE_DIR}/gnuplot-iostream")
target_link_libraries(gnuplot_iostream INTERFACE
    Boost::iostreams
    Boost::system
    Boost::filesystem
)

set(ExtObserverLibInclude "${CMAKE_SOURCE_DIR}/include/ExtObserverLib_v3")
set(ExtObserverLibSrc "${CMAKE_SOURCE_DIR}/src/ExtObserverLib_v3")

set(SOURCE
    ${ExtObserverLibSrc}/external_observer_rnea.cpp
    ${ExtObserverLibSrc}/momentum_observer_rnea.cpp
    ${ExtObserverLibSrc}/filtered_dyn_observer_rnea.cpp
    ${ExtObserverLibSrc}/nonlinear_disturbance_observer_rnea.cpp
    ${ExtObserverLibSrc}/sliding_mode_observer_rnea.cpp
    ${ExtObserverLibSrc}/kalman_filter_observer_rnea.cpp
    ${ExtObserverLibSrc}/kalman_filter_zero_order.cpp
    ${ExtObserverLibSrc}/kalman_filter_zero_order_rnea.cpp
    src/UR5Simulation/observers.cpp
    src/UR5Simulation/UR5SimulationModel.cpp
    src/UR5EstimateForceEstimation.cpp)

add_executable(UR5EstimateForceEstimation  ${SOURCE})
target_include_directories(UR5EstimateForceEstimation PUBLIC ${ExtObserverLibInclude} PUBLIC include PUBLIC ${EIGEN3_INCLUDE_DIR})
target_link_libraries(UR5EstimateForceEstimation PUBLIC Boost::program_options gnuplot_iostream)