cmake_minimum_required(VERSION 3.11)
project(cmink LANGUAGES C)

set(MUJOCO_BUILD_EXAMPLES OFF CACHE BOOL "Disable examples" FORCE)
set(MUJOCO_BUILD_SIMULATE OFF CACHE BOOL "Disable UI/simulate" FORCE)
set(MUJOCO_BUILD_TESTS OFF CACHE BOOL "Disable tests" FORCE)
set(MUJOCO_TEST_PYTHON_UTIL OFF CACHE BOOL "Disable python utils" FORCE)

find_package(libxml2 REQUIRED)

add_library(${PROJECT_NAME} 
  src/chain.c
  src/geometry3d.c
)

target_include_directories(${PROJECT_NAME} PUBLIC 
  include 
  ${LIBXML2_INCLUDE_DIR}
)

target_link_libraries(${PROJECT_NAME} PUBLIC 
  LibXml2::LibXml2
)

# debug exe
add_executable(main src/main.c)
target_link_libraries(main PRIVATE ${PROJECT_NAME})
