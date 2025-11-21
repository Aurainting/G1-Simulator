cmake_minimum_required(VERSION 3.16)
project(g1-simulator LANGUAGES CXX)

if (CMAKE_BUILD_TYPE MATCHES "Debug")
  message("Debug mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -g -O0 -fPIC")
else()
  message("Release mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3 -DNDEBUG -fPIC")
endif()

# Use pixi to manage cpp packages
set(PIXI_DIR ${CMAKE_SOURCE_DIR}/.pixi/envs/default)

find_library(GLFW_LIB
    NAMES glfw
    PATHS ${PIXI_DIR}/lib
    NO_DEFAULT_PATH
)

find_library(MuJoCo_LIB
    NAMES mujoco
    PATHS ${PIXI_DIR}/lib
    NO_DEFAULT_PATH
)

find_library(YAML_LIB
    NAMES yaml-cpp
    PATHS ${PIXI_DIR}/lib
    NO_DEFAULT_PATH
)

find_library(BOOST_LIB
    NAMES boost_program_options
    PATHS ${PIXI_DIR}/lib
    NO_DEFAULT_PATH
)

find_library(FMT_LIB
    NAMES fmt
    PATHS ${PIXI_DIR}/lib
    NO_DEFAULT_PATH
)

# Deal with unitree_sdk2
set(BUILD_EXAMPLES OFF CACHE BOOL "Build the unitree_sdk2 examples" FORCE)

add_subdirectory(${CMAKE_SOURCE_DIR}/3rdparty/unitree_sdk2)

target_include_directories(unitree_sdk2 INTERFACE
    ${PIXI_DIR}/include
)

# External sources
file(GLOB EXTERNAL_SRC
    ${CMAKE_SOURCE_DIR}/3rdparty/joystick/joystick.cc
    ${CMAKE_SOURCE_DIR}/3rdparty/lodepng/lodepng.cpp
    ${CMAKE_SOURCE_DIR}/3rdparty/mujoco-simulate/glfw_*.cc
    ${CMAKE_SOURCE_DIR}/3rdparty/mujoco-simulate/platform_*.cc
    ${CMAKE_SOURCE_DIR}/3rdparty/mujoco-simulate/simulate.cc
)

# My sources
file(GLOB MY_SRC ${CMAKE_SOURCE_DIR}/simulate/src/*.cc)

# Set my target
add_executable(${PROJECT_NAME}
    ${EXTERNAL_SRC}
    ${MY_SRC}
    ${CMAKE_SOURCE_DIR}/main.cc
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/simulate/inc
    ${PIXI_DIR}/include
    ${PIXI_DIR}/include/GLFW
    ${PIXI_DIR}/include/mujoco
    ${CMAKE_SOURCE_DIR}/3rdparty/joystick
    ${CMAKE_SOURCE_DIR}/3rdparty/lodepng
    ${CMAKE_SOURCE_DIR}/3rdparty/mujoco-simulate
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    unitree_sdk2
    ${GLFW_LIB}
    ${MuJoCo_LIB}
    ${YAML_LIB}
    ${BOOST_LIB}
    ${FMT_LIB}
)
