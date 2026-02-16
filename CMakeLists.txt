cmake_minimum_required(VERSION 3.16)
project(dragonfly_flight_control LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_compile_options(-Wall -Wextra -Wpedantic)

# Find dependencies
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(HDF5 REQUIRED COMPONENTS C CXX)

# Fetch HighFive (header-only HDF5 C++ wrapper)
include(FetchContent)
FetchContent_Declare(
    HighFive
    GIT_REPOSITORY https://github.com/highfive-devs/highfive.git
    GIT_TAG v3.1.1
)
set(HIGHFIVE_USE_EIGEN ON CACHE BOOL "Enable Eigen support in HighFive")
set(HIGHFIVE_EXAMPLES OFF CACHE BOOL "Disable HighFive examples")
set(HIGHFIVE_UNIT_TESTS OFF CACHE BOOL "Disable HighFive tests")
FetchContent_MakeAvailable(HighFive)

# Tests
enable_testing()

# Find NLopt
find_package(NLopt REQUIRED)

# Physics library (shared between main executable and tests)
add_library(dragonfly_physics STATIC
    src/rotation.cpp
    src/blade_element.cpp
    src/wing.cpp
    src/eom.cpp
    src/integrator.cpp
    src/config.cpp
    src/terminal_velocity.cpp
    src/sim_setup.cpp
)

target_include_directories(dragonfly_physics PUBLIC
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(dragonfly_physics PUBLIC
    Eigen3::Eigen
)

# Sampling library (Sobol sequence generator)
add_library(dragonfly_sampling STATIC
    src/sampling.cpp
)

target_link_libraries(dragonfly_sampling PUBLIC
    dragonfly_physics
)

# Controller library
add_library(dragonfly_controller STATIC
    src/controller.cpp
    src/trajectory.cpp
    src/trajectory_controller.cpp
)

target_link_libraries(dragonfly_controller PUBLIC
    dragonfly_physics
)

# Optimizer library
add_library(dragonfly_optim STATIC
    src/optimize.cpp
)

target_link_libraries(dragonfly_optim PUBLIC
    dragonfly_physics
    dragonfly_sampling
    HighFive
    ${HDF5_LIBRARIES}
)

target_include_directories(dragonfly_optim PUBLIC
    ${HDF5_INCLUDE_DIRS}
)

# I/O library (HDF5 simulation output)
add_library(dragonfly_io STATIC
    src/output.cpp
)

target_link_libraries(dragonfly_io PUBLIC
    dragonfly_physics
    HighFive
    ${HDF5_LIBRARIES}
)

target_include_directories(dragonfly_io PUBLIC
    ${HDF5_INCLUDE_DIRS}
)

# CLI executable
add_executable(dragonfly_cli
    apps/dragonfly_cli.cpp
    apps/cmd_sim.cpp
    apps/cmd_track.cpp
    apps/cmd_optim.cpp
    apps/cmd_plot.cpp
    apps/cmd_termvel.cpp
    apps/cmd_wingtest.cpp
)
target_include_directories(dragonfly_cli PRIVATE
    ${CMAKE_SOURCE_DIR}/apps
)
target_link_libraries(dragonfly_cli PRIVATE
    dragonfly_io
    dragonfly_optim
    dragonfly_controller
    NLopt::nlopt
)
set_target_properties(dragonfly_cli PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
    OUTPUT_NAME dragonfly
)

function(add_dragonfly_test target test_name library)
    add_executable(${target} tests/${target}.cpp)
    target_link_libraries(${target} PRIVATE ${library})
    set_target_properties(${target} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
    )
    add_test(NAME ${test_name} COMMAND ${target})
endfunction()

add_dragonfly_test(test_freefall          FreeFall         dragonfly_physics)
add_dragonfly_test(test_blade_element     BladeElement     dragonfly_physics)
add_dragonfly_test(test_rotation          Rotation         dragonfly_physics)
add_dragonfly_test(test_stationary_wing   StationaryWing   dragonfly_physics)
add_dragonfly_test(test_optimizer         Optimizer        dragonfly_optim)
add_dragonfly_test(test_sampling          Sampling         dragonfly_sampling)
add_dragonfly_test(test_terminal_velocity TerminalVelocity dragonfly_physics)
add_dragonfly_test(test_pid              PID              dragonfly_controller)
add_dragonfly_test(test_config           Config           dragonfly_physics)

add_executable(test_tracking
    tests/test_tracking.cpp
    apps/cmd_track.cpp
)
target_include_directories(test_tracking PRIVATE
    ${CMAKE_SOURCE_DIR}/apps
)
target_link_libraries(test_tracking PRIVATE
    dragonfly_io
    dragonfly_optim
    dragonfly_controller
    NLopt::nlopt
)
set_target_properties(test_tracking PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
add_test(NAME Tracking COMMAND test_tracking)
