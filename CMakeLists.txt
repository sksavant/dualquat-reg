project(global_dq_reg)

cmake_minimum_required(VERSION 2.8)

find_package(PCL REQUIRED)

set(dq_diffusion_HDRS
    dualquat/dual_quaternions.h
    dualquat/mesh.h
    dualquat/math3d.h)

set(dq_diffusion_SRCS
    dualquat/mesh.cpp
    dualquat/diffusion.hpp)

include_directories(
    ${PCL_INCLUDE_DIRS})

add_library(dq_diffusion SHARED ${dq_diffusion_HDRS} ${dq_diffusion_SRCS})

add_library(global_register_dq SHARED global_register.h global_register.cpp)
target_link_libraries(global_register_dq ${PCL_LIBRARIES})


add_executable(global_register main.cpp)
target_link_libraries(global_register dq_diffusion global_register_dq)
