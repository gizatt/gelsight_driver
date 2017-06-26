cmake_minimum_required(VERSION 2.8)
project(gelsight-driver)
set(CMAKE_CXX_STANDARD 11)

option(BUILD_STANDALONE "Build without LCM, RemoteTreeViewer, or Drake." OFF)

if (!BUILD_STANDALONE)
	find_package(drake REQUIRED)
endif()

find_package(OpenCV 2 REQUIRED)

include_directories(${CMAKE_INSTALL_PREFIX})
include_directories(${CMAKE_INSTALL_PREFIX}/include)

if (!BUILD_STANDALONE)
	include_directories(${CMAKE_INSTALL_PREFIX}/include/lcmtypes)
endif()

link_directories(${CMAKE_INSTALL_PREFIX}/lib)

set(EIGEN3_INCLUDE_DIR ${CMAKE_INSTALL_PREFIX}/include/eigen3)
include_directories(${EIGEN3_INCLUDE_DIR}) 

add_executable(groundtruth_gen gelsight_groundtruth_gen.cpp)
target_link_libraries(groundtruth_gen ${OpenCV_LIBS} pthread)

add_executable(sphere_groundtruth_gen gelsight_sphere_depth_gen.cpp)
target_link_libraries(sphere_groundtruth_gen ${OpenCV_LIBS} pthread)

add_executable(lookup_gen gelsight_lookup_gen.cpp)
target_link_libraries(lookup_gen ${OpenCV_LIBS} pthread)

add_executable(compare_heightmaps gelsight_compare_heightmaps.cpp)
target_link_libraries(compare_heightmaps ${OpenCV_LIBS} pthread)

add_executable(gelsight_depth_driver gelsight_depth_driver.cpp)

if (BUILD_STANDALONE)
	target_link_libraries(gelsight_depth_driver ${OpenCV_LIBS} pthread)
	set_target_properties(gelsight_depth_driver PROPERTIES COMPILE_DEFINITIONS BUILD_STANDALONE=1)
else()
	target_link_libraries(gelsight_depth_driver ${OpenCV_LIBS} drakeLcm pthread bot2-core lcmtypes_bot2-core RemoteTreeViewerWrapper)
endif()

install(TARGETS gelsight_depth_driver DESTINATION bin)
