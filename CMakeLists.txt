cmake_minimum_required(VERSION 3.16)
project(dragonfly_flight_control LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find dependencies
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(HDF5 REQUIRED COMPONENTS C CXX)

# Fetch HighFive (header-only HDF5 C++ wrapper)
include(FetchContent)
FetchContent_Declare(
    HighFive
    GIT_REPOSITORY https://github.com/highfive-devs/highfive.git
    GIT_TAG main
)
set(HIGHFIVE_USE_EIGEN ON CACHE BOOL "Enable Eigen support in HighFive")
set(HIGHFIVE_EXAMPLES OFF CACHE BOOL "Disable HighFive examples")
set(HIGHFIVE_UNIT_TESTS OFF CACHE BOOL "Disable HighFive tests")
FetchContent_MakeAvailable(HighFive)

# Tests
enable_testing()

# Physics library (shared between main executable and tests)
add_library(dragonfly_physics STATIC
    src/rotation.cpp
    src/blade_element.cpp
    src/wing.cpp
    src/eom.cpp
    src/integrator.cpp
)

target_include_directories(dragonfly_physics PUBLIC
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(dragonfly_physics PUBLIC
    Eigen3::Eigen
)

# Find NLopt
find_package(NLopt REQUIRED)

# Optimizer library
add_library(dragonfly_optim STATIC
    src/optimizer/flex_optim.cpp
)

target_include_directories(dragonfly_optim PUBLIC
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(dragonfly_optim PUBLIC
    dragonfly_physics
)

# CLI executable
add_executable(dragonfly_cli
    apps/dragonfly_cli.cpp
    apps/cmd_sim.cpp
    apps/cmd_optim.cpp
    apps/cmd_plot.cpp
    apps/cmd_wingtest.cpp
    src/output.cpp
)
target_include_directories(dragonfly_cli PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/apps
    ${HDF5_INCLUDE_DIRS}
)
target_link_libraries(dragonfly_cli PRIVATE
    dragonfly_optim
    NLopt::nlopt
    HighFive
    ${HDF5_LIBRARIES}
)
set_target_properties(dragonfly_cli PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
    OUTPUT_NAME dragonfly
)

# Free fall test
add_executable(test_freefall tests/test_freefall.cpp)
target_link_libraries(test_freefall PRIVATE dragonfly_physics)
set_target_properties(test_freefall PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
add_test(NAME FreeFall COMMAND test_freefall)

# Blade element coefficient test
add_executable(test_blade_element tests/test_blade_element.cpp)
target_link_libraries(test_blade_element PRIVATE dragonfly_physics)
set_target_properties(test_blade_element PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
add_test(NAME BladeElement COMMAND test_blade_element)

# Rotation matrix test
add_executable(test_rotation tests/test_rotation.cpp)
target_link_libraries(test_rotation PRIVATE dragonfly_physics)
set_target_properties(test_rotation PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
add_test(NAME Rotation COMMAND test_rotation)

# Stationary wing test
add_executable(test_stationary_wing tests/test_stationary_wing.cpp)
target_link_libraries(test_stationary_wing PRIVATE dragonfly_physics)
set_target_properties(test_stationary_wing PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
add_test(NAME StationaryWing COMMAND test_stationary_wing)

# Optimizer test
add_executable(test_optimizer tests/test_optimizer.cpp)
target_link_libraries(test_optimizer PRIVATE dragonfly_optim)
set_target_properties(test_optimizer PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
add_test(NAME Optimizer COMMAND test_optimizer)
