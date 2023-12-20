cmake_minimum_required(VERSION 3.16)
project(dubins_curve DESCRIPTION "a dubins curve example")

find_package(catkin REQUIRED COMPONENTS
    nav_msgs
    roscpp
    rospy
    std_msgs
    tf
)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
    "src/dubins_curve.cpp"
    "src/dubins_tool.cpp"
    "src/main.cpp"
)

add_executable(${PROJECT_NAME}_node
    src/main.cpp
)

target_link_libraries(${PROJECT_NAME}_node
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
)