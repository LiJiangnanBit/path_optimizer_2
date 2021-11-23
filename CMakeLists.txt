cmake_minimum_required(VERSION 2.8.3)
project(path_optimizer_2)

## Compile as C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11 -w)
set(CMAKE_CXX_FLAGS " -Wall -Wextra ${CMAKE_CXX_FLAGS}")
set(CMAKE_BUILD_TYPE "Release")

set(catkin_deps
        roscpp
        grid_map_ros
        tinyspline_ros
        ros_viz_tools
        )

find_package(catkin REQUIRED COMPONENTS
        ${catkin_deps}
        )

find_package(OsqpEigen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV 3 REQUIRED)
find_package(gflags REQUIRED)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS ${catkin_deps}
        DEPENDS OpenCV
)

include_directories(
        include
        ${OpenCV_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
)

add_library(${PROJECT_NAME}
        src/tools/tools.cpp
        src/tools/spline.cpp
        src/path_optimizer.cpp
        src/tools/collision_checker.cpp
        src/reference_path_smoother/reference_path_smoother.cpp
        src/tools/Map.cpp
        src/tools/car_geometry.cpp
        src/solver/base_solver.cpp
        src/data_struct/date_struct.cpp
        src/data_struct/reference_path_impl.cpp
        src/data_struct/reference_path.cpp
        src/data_struct/vehicle_state_frenet.cpp
        src/config/planning_flags.cpp
        src/reference_path_smoother/tension_smoother.cpp
        src/reference_path_smoother/tension_smoother_2.cpp
        src/tools/time_recorder.cpp)
target_link_libraries(${PROJECT_NAME} glog gflags ${catkin_LIBRARIES} OsqpEigen::OsqpEigen osqp::osqp
        )

add_executable(${PROJECT_NAME}_demo
        src/test/demo.cpp)
target_link_libraries(${PROJECT_NAME}_demo
        ${PROJECT_NAME} ${OpenCV_LIBRARIES}
        )
