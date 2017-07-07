add_executable(shear_tracking_demo main.cpp CaptureFrm.cpp MarkerTrack.cpp VideoRecord.cpp WindowDisplay.cpp)
target_link_libraries(shear_tracking_demo ${OpenCV_LIBS} pthread)

install(TARGETS shear_tracking_demo DESTINATION bin)