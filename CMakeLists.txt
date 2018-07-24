cmake_minimum_required(VERSION 3.5)
project(cycle-routing)
add_compile_options(-Wall -Wextra -Wpedantic -std=c++17 -Wno-register)

# Set a default build type if none was specified
set(default_build_type "Release")
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
  set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE
      STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
include(cotire)

find_package(glpk REQUIRED)

find_package(Boost 1.58 REQUIRED COMPONENTS serialization program_options filesystem)
find_package (Threads)

find_package(glpk REQUIRED) 


add_library(process INTERFACE)
target_include_directories(process SYSTEM INTERFACE vendor/boost-process)

# Actual Implementation of project in static library
file(GLOB lib_src src/multi_lib/*.cpp )
add_library(multi_lib STATIC ${lib_src})
target_include_directories(multi_lib PUBLIC src/multi_lib /usr/include/coin )
target_link_libraries(multi_lib ${GLPK_LIBRARIES})
target_link_libraries(multi_lib ${Boost_LIBRARIES})
target_link_libraries(multi_lib -lm)
target_link_libraries(multi_lib ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(multi_lib process)


# Executable of project links against lib
add_executable(multi-ch src/main.cpp)
target_link_libraries(multi-ch multi_lib)


add_executable(multi_lp src/lpsolver.cpp)
target_link_libraries(multi_lp ${GLPK_LIBRARIES})

cotire(multi_lib multi-ch)
