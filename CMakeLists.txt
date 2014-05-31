project(global_dq_reg)

cmake_minimum_required(VERSION 2.8)

find_package(PCL 1.7.2 REQUIRED)
find_package(Eigen REQUIRED)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(dq_diffusion_HDRS
    dualquat/dual_quaternions.h
    dualquat/math3d.h)

set (dq_diffusion_SRCS
    dualquat/mesh.cpp
    dualquat/main.cpp
    dualquat/diffusion.hpp
    )

include_directories(
    ${PCL_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
    dualquat/
    )

add_definitions("-g")

add_executable(dq_diffusion_demo ${dq_diffusion_SRCS} ${dq_diffusion_HDRS})

add_library(global_register_dq SHARED global_register.h global_register.cpp ${dq_diffusion_HDRS})
target_link_libraries(global_register_dq ${PCL_LIBRARIES})

add_executable(global_register main.cpp)
target_link_libraries(global_register global_register_dq)
