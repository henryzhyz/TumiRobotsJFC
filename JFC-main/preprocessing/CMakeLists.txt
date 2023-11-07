# CMakeList.txt : CMake project for CMakeProject1, include source and define
# project specific logic here.
#
cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(PCD_Preprocessor)


cmake_policy(SET CMP0074 NEW)
find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_definitions( -DBOOST_ALL_NO_LIB )

add_executable (PCD_Preprocessor src/PCDPreprocessor.cpp)
target_link_libraries (PCD_Preprocessor ${PCL_LIBRARIES})
