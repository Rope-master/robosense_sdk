cmake_minimum_required(VERSION 2.8.3)
project(auto_align_tool)

find_package(catkin REQUIRED
        roscpp
        cv_bridge
        pcl_conversions
        pcl_ros
        )
find_package(Qt5Widgets REQUIRED)
find_package (PCL 1.7.1 REQUIRED)
include_directories (${PCL_INCLUDE_DIRS})
link_directories    (${PCL_LIBRARY_DIRS})
add_definitions     (${PCL_DEFINITIONS})

catkin_package(
)

include_directories(
        ${catkin_INCLUDE_DIRS}
        include
        ui
        ${Qt5Widgets_INCLUDE_DIRS}
)

set(CMAKE_AUTOMOC ON)

add_library(align_mainwindow
        src/mainwindow.cpp
        include/mainwindow.h
        )
target_link_libraries(align_mainwindow ${Qt5Widgets_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(align_node
        node/main.cpp
        )
target_link_libraries(align_node align_mainwindow ${PCL_LIBRARIES})

add_executable(align_adjust_node
        node/adjust_node.cpp
        )
target_link_libraries(align_adjust_node ${PCL_LIBRARIES} ${catkin_LIBRARIES})
