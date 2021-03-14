cmake_minimum_required(VERSION 3.8)

project(iris)

# Build type.
set(default_build_type "Release")

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
    set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE STRING "Choose the type of build." FORCE)

    # Set the possible values of build type for cmake-gui
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")

    # On 32bit architectures, use RelWithDebInfo
    if (CMAKE_COMPILER_IS_GNUCC AND CMAKE_SIZEOF_VOID_P EQUAL 4)
        set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Type of build" FORCE)
    endif()
endif()

message(STATUS "Building ${CMAKE_BUILD_TYPE}")

set(USE_C++17 0)

if(USE_C++17 OR (UNIX AND NOT APPLE))
    # Use C++ 17 for nign.
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED 17)
    set(USE_C++17 1)
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")

message(STATUS "Using c++17: " ${USE_C++17})

# Check for eigen3
find_package(Eigen3 REQUIRED)
if(NOT EIGEN3_FOUND)
    set(EIGEN3_INCLUDE_DIR "EIGEN3-NOTFOUND" CACHE PATH "Location of EIGEN3 headers")
endif()

if(EIGEN3_INCLUDE_DIR)
    include_directories(${EIGEN3_INCLUDE_DIR})
    message("Eigen3 include dir: ${EIGEN3_INCLUDE_DIR}")
else(EIGEN3_INCLUDE_DIR)
    message("Eigen3 not found! Please set it manually!")
endif()


# Check for Boost
find_package(Boost 1.56.0 COMPONENTS system program_options timer chrono thread serialization date_time filesystem regex)
include_directories(${Boost_INCLUDE_DIRS})
message("Boost headers: ${Boost_INCLUDE_DIRS}")
message("Boost libraries: ${Boost_LIBRARIES}")

# Check for OMPL
find_package(OMPL QUIET)
if (NOT OMPL_FOUND AND NOT OMPL_INCLUDE_DIRS)
    set(OMPL_INCLUDE_DIRS "OMPL-NOTFOUND" CACHE PATH "Location of OMPL headers")
    set(OMPL_LIBRARIES "OMPL-NOTFOUND" CACHE FILEPATH "Location of OMPL library")
endif()

if(OMPL_INCLUDE_DIRS AND OMPL_LIBRARIES)
    include_directories(${OMPL_INCLUDE_DIRS})
    include_directories("${OMPL_INCLUDE_DIRS}/ompl-1.5")
  	message("OMPL found.")
    message("OMPL include dirs: ${OMPL_INCLUDE_DIRS}")
    message("OMPL libraries: ${OMPL_LIBRARIES}")
else(OMPL_INCLUDE_DIRS AND OMPL_LIBRARIES)
    message("Headers/Libraries not specified for OMPL; not configuring OMPL projects.")
endif(OMPL_INCLUDE_DIRS AND OMPL_LIBRARIES)


include_directories(BEFORE "include")
include_directories(BEFORE "${CMAKE_CURRENT_BINARY_DIR}/include")

include_directories(BEFORE "external")
include_directories(BEFORE "${CMAKE_CURRENT_BINARY_DIR}/external")

include_directories(BEFORE "external/nigh/src")
include_directories(BEFORE "${CMAKE_CURRENT_BINARY_DIR}/external/nigh/src")

add_subdirectory(src)

add_subdirectory(app)

add_subdirectory(external)
