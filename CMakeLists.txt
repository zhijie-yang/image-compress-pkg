cmake_minimum_required(VERSION 2.8.3)
project(image_compress_pkg)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        pcl_conversions
        pcl_ros
        roscpp
        sensor_msgs
        geometry_msgs
        serial
        message_generation
        cv_bridge
        )

find_package(OpenCV REQUIRED)

catkin_package(
#        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS cv_bridge roscpp
        DEPENDS OpenCV
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/main.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
