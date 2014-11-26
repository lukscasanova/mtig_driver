cmake_minimum_required(VERSION 2.4.6)
project(mtig_driver)

find_package(catkin REQUIRED COMPONENTS 
	roscpp 
	std_msgs 
	message_generation
	tf
)

add_message_files(
			FILES
			GpsInfo.msg
)

generate_messages(
			DEPENDENCIES
			std_msgs
)


catkin_package(
			INCLUDE_DIRS include			
			LIBRARIES ${PROJECT_NAME}
			CATKIN_DEPENDS roscpp tf
			DEPENDS message_runtime
)






# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

include_directories(include ${catkin_INCLUDE_DIRS}
  include/${PROJECT_NAME})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()



#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
#target_link_libraries(${PROJECT_NAME} libxsensdeviceapi)

#set(CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH} /opt/local/lib)
add_executable(mtig_driver_node src/main.cpp src/console.cpp src/conio.c src/myxda.cpp src/serialkey.cpp src/mtiG.cpp src/messageMaker.cpp src/sensorData.cpp)
target_link_libraries(mtig_driver_node m xsensdeviceapi xstypes pthread ${catkin_LIBRARIES})
