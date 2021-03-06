cmake_minimum_required(VERSION 2.8.3)
project(People_counter)

add_definitions(-std=c++11)
#set(CMAKE_CXX_FLAGS "-std=c++11 -O2 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    std_msgs
    image_transport
    image_geometry
    camera_info_manager
    sensor_msgs
    tf
    dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/cam.cfg
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    std_msgs
    image_transport
    image_geometry
    camera_info_manager
    sensor_msgs
    tf
    dynamic_reconfigure
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(People_counter src/oCamS.cpp src/withrobot_camera.cpp src/withrobot_utility.cpp)
target_link_libraries(People_counter ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS} -lv4l2 -ludev)
add_dependencies(People_counter ${PROJECT_NAME}_gencfg)

###############################################################################

#Add all files in subdirectories of the project in
# a dummy_target so qtcreator have access to all files
FILE(GLOB_RECURSE extra_files ${CMAKE_SOURCE_DIR}/*)
add_custom_target(dummy_${PROJECT_NAME} SOURCES ${extra_files})

###############################################################################
