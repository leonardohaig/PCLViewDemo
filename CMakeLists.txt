#SpacePointShow 的makeLists 文件
cmake_minimum_required(VERSION 3.9)
set(PROJECTNAME PCLViewDemo)
project(${PROJECTNAME})

MESSAGE(STATUS "===========================================================")
MESSAGE(STATUS "==========This projece name is " ${PROJECTNAME} "==========")
MESSAGE(STATUS "===========================================================")

set(CMAKE_CXX_STANDARD 11)


find_package(OpenCV REQUIRED)

#if (POLICY CMP0074)
#    cmake_policy(SET CMP0074 NEW)
#endif()

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


aux_source_directory(. DIR_STEREOVISIONFORADAS_SRCS)

add_executable(${PROJECTNAME} main.cpp)

target_link_libraries(${PROJECTNAME} ${OpenCV_LIBS})
target_link_libraries(${PROJECTNAME} ${PCL_LIBRARIES})