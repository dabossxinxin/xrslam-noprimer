cmake_minimum_required(VERSION 3.11 FATAL_ERROR)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")
project(xrslam-pc-player LANGUAGES CXX)

if(TARGET xrslam-extra-opencv-image AND TARGET xrslam-extra-yaml-config)

include(SuperBuildDepends)
superbuild_depend(argparse)

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
  add_definitions(-DGLOG_NO_ABBREVIATED_SEVERITIES)
endif()

add_executable(xrslam-pc-player
  ${CMAKE_CURRENT_SOURCE_DIR}/src/dataset_reader.h
  ${CMAKE_CURRENT_SOURCE_DIR}/src/dataset_reader.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/async_dataset_reader.h
  ${CMAKE_CURRENT_SOURCE_DIR}/src/async_dataset_reader.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/euroc_dataset_reader.h
  ${CMAKE_CURRENT_SOURCE_DIR}/src/euroc_dataset_reader.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/tum_dataset_reader.h
  ${CMAKE_CURRENT_SOURCE_DIR}/src/tum_dataset_reader.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/trajectory_writer.h
  ${CMAKE_CURRENT_SOURCE_DIR}/src/opencv_painter.h
  ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
)

if(XRSLAM_PC_HEADLESS_ONLY)
target_compile_definitions(xrslam-pc-player
  PRIVATE
    XRSLAM_PC_HEADLESS_ONLY
)
endif()

target_include_directories(xrslam-pc-player
  PRIVATE
	${EIGEN3_INCLUDE_DIR}
	${OpenCV4_INCLUDE_DIR}
	${GLOG_INCLUDE_DIRS}
	${CERES_INCLUDE_DIRS}
	${GFLAGS_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/src
	${CMAKE_CURRENT_SOURCE_DIR}/../lightvis/include
	${CMAKE_CURRENT_SOURCE_DIR}/../../xrslam/include
	${CMAKE_CURRENT_BINARY_DIR}/../../xrslam/include
)

target_link_libraries(xrslam-pc-player
  PRIVATE
    xrslam-extra-opencv-image
    xrslam-extra-yaml-config
    depends::argparse
	${OpenCV4_RELEASE_LIB}
	${GLOG_LIBRARIES}
	${CERES_RELEASE_LIB}
	${GFLAGS_LIBRARIES}
)

if(NOT XRSLAM_PC_HEADLESS_ONLY)
target_link_libraries(xrslam-pc-player
  PRIVATE
    depends::glbinding
    depends::nuklear
    lightvis
	${OpenCV4_RELEASE_LIB}
	${GLOG_LIBRARIES}
	${CERES_RELEASE_LIB}
	${GFLAGS_LIBRARIES}
)

endif()

endif()
