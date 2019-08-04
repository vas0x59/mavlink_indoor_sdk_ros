cmake_minimum_required(VERSION 2.8.3)
project(mavlink_indoor_sdk_sitl)


add_compile_options(-std=c++11)


# find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS 
    cv_bridge
    geometry_msgs
    image_transport
    roscpp
    sensor_msgs
    std_msgs
    tf2
)
find_package( OpenCV REQUIRED )

include_directories("./src/mavlink_indoor_sdk/mavlink/v2.0/common")

# Markers
include_directories("./src/markers_lib/src")

#SDK
include_directories("./src/mavlink_indoor_sdk/src")


catkin_package(

)

include_directories(
# include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)



add_executable(udp_vpe_sitl src/udp_vpe_sitl.cpp)

target_link_libraries(udp_vpe_sitl
    #mavlink_indoor_sdk
    pthread
    ${PROJECT_SOURCE_DIR}/src/mavlink_indoor_sdk/libmavlink_indoor_sdk.so
    ${PROJECT_SOURCE_DIR}/src/markers_lib/libmarkers_lib.so
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
)