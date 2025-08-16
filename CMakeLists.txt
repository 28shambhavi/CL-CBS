cmake_minimum_required(VERSION 3.5)

project(CL-CBS)

# Find Python3 with required components for pybind11
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

set(Boost_USE_STATIC_LIBS ON)
find_package(Boost REQUIRED COMPONENTS program_options)
message(STATUS "Boost_LIBRARIES: ${Boost_LIBRARIES}")
message(STATUS "Boost_INCLUDE_DIRS: ${Boost_INCLUDE_DIRS}")

find_package(PkgConfig)
pkg_check_modules(YamlCpp yaml-cpp)

include_directories(${YamlCpp_INCLUDE_DIRS})
link_directories(${YamlCpp_LIBRARY_DIRS})

## OPEN MOTION PLANNING LIBRARY
find_package(OMPL REQUIRED)

if(NOT OMPL_FOUND)
    message(AUTHOR_WARNING,"Open Motion Planning Library not found")
endif(NOT OMPL_FOUND)

# check if Doxygen is installed
find_package(Doxygen)

if (DOXYGEN_FOUND)
  # set input and output files
  set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/doxygen/Doxyfile.in)
  set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

  # request to configure the file
  configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
  message("Doxygen build started")

  # note the option ALL which allows to build the docs together with the application
  add_custom_target( docs
    COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/docs
    COMMENT "Generating API documentation with Doxygen"
    VERBATIM )
else (DOXYGEN_FOUND)
  message("Doxygen need to be installed to generate the doxygen documentation")
endif (DOXYGEN_FOUND)

add_subdirectory(extern/pybind11)

# Enable C++14 and warnings
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if (${CMAKE_BUILD_TYPE} MATCHES "Release")
  message("Release mode")
  set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -Wextra -O3")
else(${CMAKE_BUILD_TYPE} MATCHES "Release")
  set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -Wextra")
endif (${CMAKE_BUILD_TYPE} MATCHES "Release") 

# Creates compile database used by clang-tidy.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include_directories(
  include
  ${OMPL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# Examples

## Spatiotemporal Hybrid-State A_star
add_executable(SH_Astar
  src/sh_astar.cpp
)
target_link_libraries(SH_Astar
  ${Boost_LIBRARIES}
  ${OMPL_LIBRARIES}
)

## CL-CBS
add_executable(CL-CBS
  src/cl_cbs.cpp
)
target_link_libraries(CL-CBS PRIVATE
  /opt/homebrew/lib/libboost_program_options.dylib
  ${OMPL_LIBRARIES}
  ${YamlCpp_LIBRARIES}
)

# Python bindings - build this AFTER the main executable
pybind11_add_module(cl_cbs_python
    src/python_bindings.cpp
)

# Important: Don't link the main cl_cbs.cpp file to avoid main() conflicts
target_include_directories(cl_cbs_python PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${Boost_INCLUDE_DIRS}
    ${OMPL_INCLUDE_DIRS}
)

target_link_libraries(cl_cbs_python PRIVATE
    ${YamlCpp_LIBRARIES}
)

# Make sure the Python module is named 'cl_cbs'
set_target_properties(cl_cbs_python PROPERTIES OUTPUT_NAME cl_cbs)

# clang-format
set(ALL_SOURCE_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/include/hybrid_astar.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/cl_cbs.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/neighbor.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/planresult.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/timer.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/environment.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/instance.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/sh_astar.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cl_cbs.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/python_bindings.cpp
)

add_custom_target(clang-format
  COMMAND clang-format -style=Google -i ${ALL_SOURCE_FILES} 
)

add_custom_target(everything
  DEPENDS clang-format docs 
)