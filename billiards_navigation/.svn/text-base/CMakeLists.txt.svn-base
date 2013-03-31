cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

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

rosbuild_download_data(http://pr.willowgarage.com/data/billiards_navigation/table-1cm.pgm maps/table-1cm.pgm 92e9294363d1bfa20c13c2500a13310d)
rosbuild_download_data(http://pr.willowgarage.com/data/billiards_navigation/table-only-legs-1cm.pgm maps/table-only-legs-1cm.pgm b9de551ce6abbbad264ee9cb8df4465d)
rosbuild_download_data(http://pr.willowgarage.com/data/billiards_navigation/table-only-legs.pgm maps/table-only-legs.pgm bbd44924f1bbeaa36bb88bb2ccc18323)
rosbuild_download_data(http://pr.willowgarage.com/data/billiards_navigation/table.pgm maps/table.pgm d6f1b4e5d6f2eaf5c4abd8669aef3dc3)

