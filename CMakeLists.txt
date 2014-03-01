project(global_dq_reg)

cmake_minimum_required(VERSION 2.8)

find_package(PCL REQUIRED)

set(dq_diffusion_HDRS
    dualquat/dual_quaternions.h
    dualquat/mesh.h
    dualquat/math3d.h)

set(dq_diffusion_SRCS
    dualquat/mesh.cpp
    dualquat/main.cpp)

add_library(dq_diffusion SHARED ${dq_diffusion_HDRS} ${dq_diffusion_SRCS})

include_directories(
    ${PCL_INCLUDE_DIRS})

add_executable(global_register global_register.cpp global_register.h)
target_link_libraries(global_register ${PCL_LIBRARIES} dq_diffusion)
