cmake_minimum_required(VERSION 2.8)
project(gelsight-driver)
set(CMAKE_CXX_STANDARD 11)
include(ExternalProject)

option(BUILD_EIGEN "Build our own copy of Eigen 3.3" ON)
option(BUILD_STANDALONE "Build without LCM, RemoteTreeViewer, or Drake." ON)

# By default, don't install out to system.
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE STRING
    "Prefix for installation of sub-packages (note: required during build!)"
    FORCE)
endif()

include_directories(${CMAKE_INSTALL_PREFIX}/include) 

if (BUILD_EIGEN)
	ExternalProject_Add( Eigen3
	  URL "http://bitbucket.org/eigen/eigen/get/3.3.tar.gz"
	  UPDATE_COMMAND ""
	  CONFIGURE_COMMAND ""
	  BUILD_COMMAND ""
	  INSTALL_COMMAND
	    ${CMAKE_COMMAND} -E copy_directory
	      ${CMAKE_BINARY_DIR}/Eigen3-prefix/src/Eigen3/Eigen
	      ${CMAKE_INSTALL_PREFIX}/include/Eigen3/Eigen &&
	    ${CMAKE_COMMAND} -E copy_directory
	      ${CMAKE_BINARY_DIR}/Eigen3-prefix/src/Eigen3/unsupported
	      ${CMAKE_INSTALL_PREFIX}/include/Eigen3/unsupported
	)
	set(EIGEN3_INCLUDE_DIR ${CMAKE_INSTALL_PREFIX}/include/Eigen3 )
else()
	list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/modules")
	find_package(Eigen3 3.3 REQUIRED)
endif()
include_directories(${EIGEN3_INCLUDE_DIR}) 

# OpenCV is used for webcam interfacing and image processing
# (and must be system-installed.)
find_package(OpenCV 2 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# libkdtree is used in the depth reconstruction
ExternalProject_Add(libkdtree
	GIT_REPOSITORY https://github.com/nvmd/libkdtree.git
	GIT_TAG 7bb7e830d6899214e9e896f920483ddb39c43f7b
    CMAKE_CACHE_ARGS
      -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}/include/libkdtree
      -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
)

# LCM and drake/director provide visualization and interop
if (!BUILD_STANDALONE)
	find_package(drake REQUIRED)
	include_directories(${CMAKE_INSTALL_PREFIX}/include/lcmtypes)
	include_directories(${CMAKE_INSTALL_PREFIX})
	include_directories(${CMAKE_INSTALL_PREFIX}/include)
endif()

add_subdirectory(src)