cmake_minimum_required(VERSION 3.20)
project(StructureFromMotionTest)

set(CMAKE_CXX_STANDARD 20)

# Detect clang. Not officially reported by cmake.
execute_process(COMMAND "${CMAKE_CXX_COMPILER}" "-v" ERROR_VARIABLE CXX_VER_STDERR)
if ("${CXX_VER_STDERR}" MATCHES ".*clang.*")
    set (CMAKE_COMPILER_IS_CLANGXX 1)
endif ()

if (CMAKE_COMPILER_IS_CLANGXX)
    message("Compiling using clang")
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
    message("Compiling using msvc")
    set(warnings "/W4 /WX /EHsc")
else()
    message( FATAL_ERROR "This project should only be compiled with clang or msvc. Currently detected ${CMAKE_CXX_COMPILER_ID}." )
endif()

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

find_package(Ceres REQUIRED)

find_package(PCL REQUIRED COMPONENTS common io filters features surface)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

message(" ")
if (CMAKE_BUILD_TYPE MATCHES Debug)
    message("DEBUG BUILD")
    add_definitions(
            -DDEBUG
    )
elseif(CMAKE_BUILD_TYPE MATCHES Release)
    message("RELEASE BUILD")
else()
    message( FATAL_ERROR "Build type not recognised: ${CMAKE_BUILD_TYPE}." )
endif ()
message(" ")

add_executable(
        StructureFromMotionTest
        tests/example.cpp
        tests/test_pose.cpp
        tests/test_simple_reprojection_error.cpp
        tests/constants.h
        tests/test_point_cloud.cpp
        tests/triangulation/test_midpoint_triangulator.cpp
        evaluation/triangulation_comparison.cpp
        qual_tests/triangulation/qual_midpoint_triangulator.cpp)

set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

if (CMAKE_COMPILER_IS_CLANGXX)
    target_compile_options(StructureFromMotionTest PRIVATE "-Weverything")
    target_compile_options(StructureFromMotionTest PRIVATE "-Wno-c99-extensions")
    target_compile_options(StructureFromMotionTest PRIVATE "-Wno-c++98-compat")
    target_compile_options(StructureFromMotionTest PRIVATE "-Wno-c++98-compat-pedantic")
    target_compile_options(StructureFromMotionTest PRIVATE "-Wno-padded")
endif()

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

target_sources(
    StructureFromMotionTest
    PUBLIC
)

target_include_directories(
    StructureFromMotionTest
    PUBLIC
    "${CMAKE_SOURCE_DIR}"
    /Users/meganfinch/opencv-4.5.3/include
    /Users/meganfinch/opencv-4.5.3/cmake-build-release
    /Users/meganfinch/opencv-4.5.3/modules/calib3d/include
    /Users/meganfinch/opencv-4.5.3/modules/core/include
    /Users/meganfinch/opencv-4.5.3/modules/features2d/include
    /Users/meganfinch/opencv-4.5.3/modules/highgui/include
    /Users/meganfinch/opencv-4.5.3/modules/flann/include
    /Users/meganfinch/opencv-4.5.3/modules/imgcodecs/include
    /Users/meganfinch/opencv-4.5.3/modules/videoio/include
    /Users/meganfinch/googletest/googletest
    /Users/meganfinch/googletest/googletest/include
    /Users/meganfinch/CLionProjects/StructureFromMotion/headers/
    /Users/meganfinch/ceres-solver/include
    /Users/meganfinch/ceres-solver/cmake-build-release/config
)

target_link_directories(
    StructureFromMotionTest
    PUBLIC
    ${CMAKE_SOURCE_DIR}
    "/Users/meganfinch/opencv-4.5.3/cmake-build-release/lib"
    /Users/meganfinch/CLionProjects/StructureFromMotion/cmake-build-release/
)

target_link_libraries(
    StructureFromMotionTest
    Threads::Threads
    opencv_calib3d
    opencv_core
    opencv_dnn
    opencv_features2d
    opencv_flann
    opencv_gapi
    opencv_highgui
    opencv_imgcodecs
    opencv_imgproc
    opencv_ml
    opencv_objdetect
    opencv_photo
    opencv_stitching
    opencv_videoio
    opencv_video
    gtest
    gtest_main
    SFMLib
    ceres
    ${PCL_LIBRARIES}
)