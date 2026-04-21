cmake_minimum_required(VERSION 3.11)
project(cmink LANGUAGES C CXX)

set(MUJOCO_BUILD_EXAMPLES OFF CACHE BOOL "Disable examples" FORCE)
set(MUJOCO_BUILD_SIMULATE OFF CACHE BOOL "Disable UI/simulate" FORCE)
set(MUJOCO_BUILD_TESTS OFF CACHE BOOL "Disable tests" FORCE)
set(MUJOCO_TEST_PYTHON_UTIL OFF CACHE BOOL "Disable python utils" FORCE)

add_subdirectory(mujoco)
add_executable(main src/main.c)
include_directories(include)
target_link_libraries(main PRIVATE mujoco::mujoco)
