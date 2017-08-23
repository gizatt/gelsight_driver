find_package(yaml-cpp)

add_library(gelsight_depth_driver gelsight_depth_driver.cc)
target_link_libraries(gelsight_depth_driver yaml-cpp ${OpenCV_LIBS} pthread)

add_executable(run_gelsight_depth_driver run_gelsight_depth_driver.cc)
target_link_libraries(run_gelsight_depth_driver gelsight_depth_driver yaml-cpp ${OpenCV_LIBS} pthread)

install(TARGETS run_gelsight_depth_driver DESTINATION bin)

if (BUILD_SHEAR_APP)
	add_subdirectory(shear_app)
endif()