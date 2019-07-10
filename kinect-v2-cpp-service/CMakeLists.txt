    
cmake_minimum_required(VERSION 3.3)
project(KinectV2)

set (CMAKE_CXX_STANDARD 11)
set( CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}" ${CMAKE_MODULE_PATH} )



find_package(RobotRaconteur REQUIRED)
find_package(KinectSDK2 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(Flann)

include_directories(${RobotRaconteur_INCLUDE_DIR} ${CMAKE_CURRENT_BINARY_DIR})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

ROBOTRACONTEUR_GENERATE_THUNK(RR_THUNK_SRCS RR_THUNK_HDRS 
	kinect2_interface.robdef
	MASTER_HEADER
	IMPORT_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/../robdef
	)

add_executable(KinectV2
	${RR_THUNK_HDRS}
	${RR_THUNK_SRCS}
	main.cpp
	kinect2_impl.cpp
	kinect2_impl.h)

if(KinectSDK2_FOUND)
  include_directories( ${KinectSDK2_INCLUDE_DIRS} )
  link_directories( ${KinectSDK2_LIBRARY_DIRS} )
  target_link_libraries( KinectV2 ${KinectSDK2_LIBRARIES} )
  
  add_custom_command( TARGET KinectV2 POST_BUILD ${KinectSDK2_COMMANDS} )
endif()	
	
target_link_libraries(KinectV2 ${PCL_LIBRARIES})
target_link_libraries(KinectV2 RobotRaconteurCore ${Boost_LIBRARIES} ${RobotRaconteurCore_Extra_libraries} )