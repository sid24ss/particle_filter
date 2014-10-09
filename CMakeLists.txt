# CMake root for the Particle Filter
# Version required.
cmake_minimum_required (VERSION 2.8)
# Project name. Let's call it PF for Particle Filter.
project (PF)

set (CMAKE_BUILD_TYPE "Release")

if( NOT DEFINED CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "" )
  message( STATUS "No build type selected, default to Release" )
  set( CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE )
endif()

message( STATUS "Build configuration: ${CMAKE_BUILD_TYPE}" )

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

set( LIBRARY_OUTPUT_PATH "${PROJECT_SOURCE_DIR}/lib" CACHE PATH
    "Target directory for all libraries." )
set( EXECUTABLE_OUTPUT_PATH "${PROJECT_SOURCE_DIR}/bin" CACHE PATH
    "Target directory for all executables." )

add_definitions( -Wall -std=c++0x )

# Find packages we will be using

# OpenCV for display
find_package( OpenCV 2 REQUIRED)
message( STATUS "OpenCV found at: ${OpenCV_INCLUDE_DIRS}" )
include_directories( ${OpenCV_INCLUDE_DIRS} )
list( APPEND PF_LIBRARIES ${OpenCV_LIBS} )

# Eigen for linear algebra and matrices
find_package( Eigen REQUIRED )
message( STATUS "Eigen found at: ${EIGEN_INCLUDE_DIRS}" )
include_directories( ${EIGEN_INCLUDE_DIRS} )

# Threading and Boost utilities
find_package( Threads REQUIRED )
list( APPEND PF_LIBRARIES ${CMAKE_THREAD_LIBS_INIT} )

FIND_PACKAGE( OpenMP REQUIRED)
if(OPENMP_FOUND)
message("OPENMP FOUND")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
include_directories( ${OPENMP_INCLUDE_DIRECTORIES})
list( APPEND PF_LIBRARIES ${OPENMP_LIBS})

endif()

find_package( Boost REQUIRED )
include_directories( ${Boost_INCLUDE_DIRS} )
list( APPEND PF_LIBRARIES ${Boost_LIBRARIES} )
list( APPEND PF_LIBRARIES "-lboost_thread" ) # Need to force linking against threads

set( PF_INCLUDE_DIRECTORIES "${PROJECT_SOURCE_DIR}/include" )
include_directories(${PF_INCLUDE_DIRECTORIES})

# Add files here to compile them
add_library(pf  SHARED  src/Map.cpp
                        src/Visualizer.cpp
                        src/Log.cpp
                        src/RobotState.cpp
                        src/MotionModel.cpp
                        src/Particle.cpp
                        src/SensorModel.cpp
                        src/ParticleFilter.cpp
                        src/Resampler.cpp
                        src/Test.cpp
                    )

set_target_properties(pf PROPERTIES OUTPUT_NAME pf)

target_link_libraries(pf ${PF_LIBRARIES})

# Add files here to make executables out of them
add_executable(test src/run_tests.cpp)
target_link_libraries(test pf ${PF_LIBRARIES})

add_executable(run_filter src/run_filter.cpp)
target_link_libraries(run_filter pf ${PF_LIBRARIES})

# add_subdirectory( src )

# add_subdirectory( app )
