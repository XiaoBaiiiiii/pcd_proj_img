cmake_minimum_required(VERSION 3.0)
project(pcd_proj_img)

option(OPENMP_ENABLED "Whether to enable OpenMP parallelization" ON)

# Find required packages
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
if(OPENMP_ENABLED)
    find_package(OpenMP REQUIRED)
endif()

# Include directories for header files
include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
)

if(OPENMP_ENABLED AND OPENMP_FOUND)
    message(STATUS "Enabling OpenMP support")
    add_definitions("-DOPENMP_ENABLED")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
else()
    message(STATUS "Disabling OpenMP support")
endif()

set(PROJECT_SOURCES
  src/pcd_proj_img.cc
)

# Define executable target
add_executable(pcd_proj_img main.cc ${PROJECT_SOURCES})

# Link libraries to the executable target
target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    yaml-cpp
)

# Set C++ version
set_property(TARGET pcd_proj_img PROPERTY CXX_STANDARD 11)