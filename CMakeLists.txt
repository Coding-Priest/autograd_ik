cmake_minimum_required(VERSION 3.11)
project(cmink LANGUAGES C)

find_package(libxml2 REQUIRED)

add_library(${PROJECT_NAME} 
  src/chain.c
  src/ops.c
  src/fk.c
)
target_compile_definitions(${PROJECT_NAME} PRIVATE MAX_URDF_SIZE=20000)
# remember to remove this
target_compile_options(${PROJECT_NAME} PRIVATE -g -fsanitize=address -fno-omit-frame-pointer -Wall)
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
target_compile_options(main PRIVATE -g -fsanitize=address -fno-omit-frame-pointer -Wall)
target_link_options(main PRIVATE -fsanitize=address)
