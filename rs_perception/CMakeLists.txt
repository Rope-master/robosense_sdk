cmake_minimum_required( VERSION 2.8.3 )
project( rs_perception )

SET( CMAKE_BUILD_TYPE "Release" )

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

OPTION(USE_DEBUG "Build the project using debug info" ON)

if (USE_DEBUG)
	add_definitions("-DUSE_DEBUG")
endif(USE_DEBUG)

find_package( catkin REQUIRED COMPONENTS
              cv_bridge
              genmsg
              pcl_conversions
              pcl_ros
              roscpp
              rospy
              std_msgs
		      geometry_msgs
              message_generation
              )
find_package( Boost REQUIRED thread signals )
find_package( PkgConfig )
find_package( OpenCV REQUIRED )

## find PCL
find_package(PCL REQUIRED)
include_directories (${PCL_INCLUDE_DIRS})

#find openssl
find_package(OpenSSL REQUIRED)
include_directories (${OPENSSL_INCLUDE_DIR})
link_directories    (${OPENSSL_LIBRARIES})

add_message_files(
		FILES
		Localization.msg
		PerceptionMsg.msg
		PerceptionListMsg.msg
)

generate_messages(
		DEPENDENCIES
		std_msgs
		geometry_msgs
)

catkin_package(
		CATKIN_DEPENDS  std_msgs geometry_msgs message_runtime
)


include_directories(
	include
    demo
    tools
	proto
    ${catkin_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)
# pcap
set( libpcap_LIBRARIES -lpcap )

# protobuf
find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIRS})
link_directories(${PROTOBUF_LIBRARY})

# sdk libs
link_directories(lib)
#file( GLOB libc lib/*.so )

add_definitions( ${EIGEN_DEFINITIONS} )
add_definitions( ${PCL_DEFINITIONS} )

## find OpenMP
find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
    message("OPENMP FOUND")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

add_library( rs_tools
		tools/display/robo_draw_rviz.cpp
        tools/communication/comm_util.hpp
		tools/communication/communication.cpp
        tools/communication/communication2.cpp
		tools/util/robosense_util.cpp
		tools/ros_message/gen_ros_msgs.hpp
		proto/perceptron.pb.h
		proto/perceptron.pb.cc
)
target_link_libraries( rs_tools rs_common ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PROTOBUF_LIBRARY} ) #


set(RS_DEPENDS rs_tools rs_robosense rs_tracking rs_classification rs_classification_base0 rs_classification_base1 rs_detection
		rs_preprocessing rs_common rs_module_manager ${catkin_LIBRARIES} ${OpenCV_LIBS} ${libpcap_LIBRARIES} ${OPENSSL_LIBRARIES}) #

# test without map
add_executable( test_without_map
                demo/test_without_map.cpp
                )
add_dependencies(test_without_map ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries( test_without_map ${RS_DEPENDS})


# test with map
add_executable( test_with_map
                demo/test_with_map.cpp
				)
add_dependencies(test_with_map ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries( test_with_map ${RS_DEPENDS} )


# test without map with p3
add_executable( p3_test_without_map
		demo/p3_test_without_map.cpp
		)
add_dependencies(p3_test_without_map ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries( p3_test_without_map ${RS_DEPENDS})


# test with map with p3
add_executable( p3_test_with_map
		demo/p3_test_with_map.cpp
		)
add_dependencies(p3_test_with_map ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries( p3_test_with_map ${RS_DEPENDS})


# test receive perception results
add_executable( test_receiver
		demo/test_receiver.cpp
		)
add_dependencies(test_receiver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries( test_receiver rs_tools rs_common)
