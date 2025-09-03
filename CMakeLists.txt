cmake_minimum_required(VERSION 3.15)
project(UR5SimulationForceEstimation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE "Debug")
set(EXECUTABLE_OUTPUT_PATH "${CMAKE_SOURCE_DIR}/bin")
set(UR_RTDE_BUILD_DIR "${CMAKE_SOURCE_DIR}/build/ur_rtde")

if(DEFINED ENV{PYTHON_VERSION})
    set(PYTHON_VERSION "$ENV{PYTHON_VERSION}")
else()
    set(PYTHON_VERSION "3.10")
endif()
set(PYTHON_VERSION ${PYTHON_VERSION} CACHE STRING "Python version to use")

file(MAKE_DIRECTORY "${UR_RTDE_BUILD_DIR}")
execute_process(
    COMMAND ${CMAKE_COMMAND}  -D PYBIND11_PYTHON_VERSION=${PYTHON_VERSION} -S "${CMAKE_SOURCE_DIR}/ur_rtde" -B "${UR_RTDE_BUILD_DIR}" 
    WORKING_DIRECTORY "${UR_RTDE_BUILD_DIR}"
    RESULT_VARIABLE ur_rtde_cmake_result
)
if(NOT ur_rtde_cmake_result EQUAL 0)
    message(FATAL_ERROR "Failed to cmake the ur_rtde during configuration")
endif()
execute_process(
    COMMAND make
    WORKING_DIRECTORY "${UR_RTDE_BUILD_DIR}"
    RESULT_VARIABLE ur_rtde_make_result
)
if(NOT ur_rtde_make_result EQUAL 0)
    message(FATAL_ERROR "Failed to make the ur_rtde during compilation.")
endif()

list(APPEND CMAKE_PREFIX_PATH "${UR_RTDE_BUILD_DIR}")
find_package(ur_rtde REQUIRED)

find_package(Boost REQUIRED COMPONENTS program_options iostreams system filesystem)
find_package(Eigen3 REQUIRED)

add_library(gnuplot_iostream INTERFACE)
target_include_directories(gnuplot_iostream INTERFACE "${CMAKE_SOURCE_DIR}/gnuplot-iostream")
target_link_libraries(gnuplot_iostream INTERFACE
    Boost::iostreams
    Boost::system
    Boost::filesystem
)

set(ExtObserverLibSrc "${CMAKE_SOURCE_DIR}/src/ExtObserverLib_v3")
set(UR5Src "${CMAKE_SOURCE_DIR}/src/UR5Simulation")
set(LogSrc "${CMAKE_SOURCE_DIR}/src/Log")

set(SOURCE
    ${ExtObserverLibSrc}/external_observer_rnea.cpp
    ${ExtObserverLibSrc}/momentum_observer_rnea.cpp
    ${ExtObserverLibSrc}/filtered_dyn_observer_rnea.cpp
    ${ExtObserverLibSrc}/nonlinear_disturbance_observer_rnea.cpp
    ${ExtObserverLibSrc}/sliding_mode_observer_rnea.cpp
    ${ExtObserverLibSrc}/kalman_filter_observer_rnea.cpp
    ${ExtObserverLibSrc}/kalman_filter_zero_order.cpp
    ${ExtObserverLibSrc}/kalman_filter_zero_order_rnea.cpp
    ${LogSrc}/logger.cpp
    ${UR5Src}/UR5SimulationModel.cpp 
    ${UR5Src}/observers.cpp)

add_executable(UR5EstimateForceEstimation src/UR5EstimateForceEstimation.cpp ${SOURCE})
target_include_directories(UR5EstimateForceEstimation PUBLIC include PUBLIC ${EIGEN3_INCLUDE_DIR})
target_link_libraries(UR5EstimateForceEstimation PUBLIC Boost::program_options gnuplot_iostream)

add_executable(testImpedanceExpControlFakeWrench src/testImpedanceExpFakeWrench.cpp src/ImpedanceControl/impedanceControlExpFakeWrench.cpp ${SOURCE})
target_include_directories(testImpedanceExpControlFakeWrench PUBLIC include PUBLIC ${EIGEN3_INCLUDE_DIR})
target_link_libraries(testImpedanceExpControlFakeWrench PUBLIC ur_rtde::rtde ${Boost_SYSTEM_LIBRARY})

