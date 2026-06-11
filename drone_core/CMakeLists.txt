cmake_minimum_required(VERSION 3.16)
project(drone_core VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Threads REQUIRED)
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc videoio highgui)
find_package(Eigen3 REQUIRED NO_MODULE)

set(DRIVER_SOURCES
    src/drivers/imu_driver.cpp
    src/drivers/camera_driver.cpp
)

add_executable(drone_core src/main.cpp ${DRIVER_SOURCES})
target_include_directories(drone_core PUBLIC include)
target_link_libraries(drone_core
    Threads::Threads
    ${OpenCV_LIBS}
    Eigen3::Eigen
)
ament_target_dependencies(drone_core rclcpp sensor_msgs cv_bridge)

install(TARGETS drone_core
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config scripts
  DESTINATION share/${PROJECT_NAME})

ament_package()
