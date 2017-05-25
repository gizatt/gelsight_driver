cmake_minimum_required(VERSION 3.5)
project(gelsight-driver)
set(CMAKE_CXX_STANDARD 11)

find_package(drake REQUIRED)
find_package(OpenCV 2 REQUIRED)

include_directories(${CMAKE_INSTALL_PREFIX})
include_directories(${CMAKE_INSTALL_PREFIX}/include)
include_directories(${CMAKE_INSTALL_PREFIX}/include/lcmtypes)
link_directories(${CMAKE_INSTALL_PREFIX}/lib)

set(EIGEN3_INCLUDE_DIR ${CMAKE_INSTALL_PREFIX}/include/eigen3)
include_directories(${EIGEN3_INCLUDE_DIR}) 
message("Eigen3 include dir" ${EIGEN3_INCLUDE_DIR})

add_executable(gelsight_depth_driver gelsight_depth_driver.cpp)
target_link_libraries(gelsight_depth_driver lcm ${OpenCV_LIBS} pthread bot2-core lcmtypes_bot2-core RemoteTreeViewerWrapper)
install(TARGETS gelsight_depth_driver DESTINATION bin)
