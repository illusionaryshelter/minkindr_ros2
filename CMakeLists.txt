cmake_minimum_required(VERSION 3.10)
project(minkindr_conversions)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++20 -pthread -fexceptions -g -ggdb") #c++17, for parallel programming
set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
find_package(OpenMP)
if(OPENMP_FOUND) #For Nano-GICP, Quatro #The best way to set proper compiler settings for using OpenMP in all platforms
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
else()
  message("ERROR: OpenMP could not be found.")
endif(OPENMP_FOUND)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -Wextra ${CMAKE_CXX_FLAGS} -fopenmp -O3 -ftree-vectorize -funroll-loops")

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(gflags REQUIRED)
find_package(xmlrpcpp REQUIRED)

##########
# GTESTS #
##########
#TODO

##########
# EXPORT #
##########
install(DIRECTORY "include"
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(
  include
)

ament_package()