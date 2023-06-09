cmake_minimum_required(VERSION 3.13 FATAL_ERROR)
set(CMAKE_BUILD_TYPE Release)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")
project(xrslam-superbuild)

include(SuperBuildDepends)

option(XRSLAM_DEBUG "..." ON)
option(XRSLAM_ENABLE_DEBUG_INSPECTION "..." ON)
option(XRSLAM_ENABLE_THREADING "..." ON)
option(XRSLAM_PC_HEADLESS_ONLY "..." OFF)
option(XRSLAM_IOS "..." OFF)
option(XRSLAM_TEST "..." OFF)

#set(CMAKE_CXX_FLAGS_RELEASE "/Od")
#set(CMAKE_C_FLAGS_RELEASE "/Od")

if(IOS)
  set(XRSLAM_IOS ON)
  set(XRSLAM_ENABLE_THREADING ON)
  set(XRSLAM_ENABLE_DEBUG_INSPECTION OFF CACHE BOOL "..." FORCE)
endif()

if(NOT IOS)
  set(XRSLAM_PC ON)
endif()

add_subdirectory(xrslam)
add_subdirectory(xrslam-extra)

if(XRSLAM_PC)
  add_subdirectory(xrslam-pc)
endif()
if(XRSLAM_IOS)
  add_subdirectory(xrslam-ios)
endif()
if(XRSLAM_TEST)
  add_subdirectory(xrslam-test)
endif()
