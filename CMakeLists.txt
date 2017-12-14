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

include_directories(include 
	${catkin_INCLUDE_DIRS}
	/usr/local/xsens/include
)

link_directories( /usr/local/xsens/lib )

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#set(CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH} /opt/local/lib)
add_executable(mtig_driver_node src/main.cpp src/console.cpp src/conio.c src/myxda.cpp 
src/serialkey.cpp src/mtiG.cpp src/messageMaker.cpp src/sensorData.cpp)
target_link_libraries(mtig_driver_node m xsensdeviceapi xstypes pthread ${catkin_LIBRARIES})
add_dependencies(mtig_driver_node ${catkin_EXPORTED_TARGETS})
