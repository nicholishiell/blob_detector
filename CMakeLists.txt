cmake_minimum_required(VERSION 2.8.3)
project(blob_detector)

find_package(catkin REQUIRED COMPONENTS std_msgs roscpp)
find_package(OpenCV REQUIRED)

catkin_package(
  DEPENDS OpenCV
)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(/usr/local/include/raspicam)
include_directories( ${OpenCV_INCLUDE_DIRS})

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_executable(
  blob_detector 
  src/blob_detector.cpp 
)

target_link_libraries(blob_detector /usr/local/lib/libraspicam.so)
target_link_libraries(blob_detector /usr/local/lib/libraspicam_cv.so)
target_link_libraries(blob_detector ${catkin_LIBRARIES})
target_link_libraries(blob_detector ${OpenCV_LIBRARIES})
