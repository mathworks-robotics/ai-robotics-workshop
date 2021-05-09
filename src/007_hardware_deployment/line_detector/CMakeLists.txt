cmake_minimum_required(VERSION 3.8 FATAL_ERROR)
project(line_detector LANGUAGES CUDA CXX)

# Path to weithgs for deep neural networks
add_definitions(-DMW_DL_DATA_PATH=${PROJECT_SOURCE_DIR}/src/codegen)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs 
  cv_bridge
  pluginlib
  nodelet
)
find_package(CUDA REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs cv_bridge nodelet
  LIBRARIES {$PROJECT_NAME}
)


file(GLOB_RECURSE cpp_srcs src/codegen/*.cpp)
file(GLOB_RECURSE cuda_srcs src/codegen/*.cu)

set(CMAKE_CUDA_FLAGS "-arch sm_35 -rdc=true -Xcudafe \"--diag_suppress=unsigned_compare_with_zero\" -Xcompiler -MMD,-MP -O2")
#target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_11)

# Include directories for building
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  ${CUDNN_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  src/codegen/
)

# enable c++11 (TensorRT requirement)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Link directories for linking
link_directories(
  ${CUDA_LIBRARY_DIRS}
  ${CUDNN_PATH} ${CUDNN_PATH}/lib ${CUDNN_PATH}/lib64
)

add_library(${PROJECT_NAME} SHARED src/nodelet/line_detector_nodelet.cpp ${cpp_srcs} ${cuda_srcs})
set_target_properties(${PROJECT_NAME}
                       PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
target_link_libraries(line_detector ${CUDA_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} cudnn cublas)

add_executable(line_detector_exe src/node/line_detector.cpp)
target_link_libraries(line_detector_exe PUBLIC ${PROJECT_NAME} ${CUDA_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
set_target_properties(line_detector_exe PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
set_target_properties(line_detector_exe PROPERTIES OUTPUT_NAME line_detector) 

install(TARGETS line_detector
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	)
install(TARGETS line_detector_exe
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	)
install(FILES nodelet_plugins.xml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
	)
install(DIRECTORY launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
	)

