cmake_minimum_required(VERSION 3.16)
project(haulroute LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

option(HAULROUTE_SANITIZE "Build with address and undefined-behavior sanitizers" OFF)

add_library(haulroute src/graph.cpp src/io.cpp src/search.cpp src/ch.cpp)
target_include_directories(haulroute PUBLIC include)
target_compile_options(haulroute PRIVATE -Wall -Wextra -Wpedantic)

if(HAULROUTE_SANITIZE)
  target_compile_options(haulroute PUBLIC -fsanitize=address,undefined -fno-omit-frame-pointer)
  target_link_options(haulroute PUBLIC -fsanitize=address,undefined)
endif()

add_executable(haulroute_cli tools/haulroute.cpp)
set_target_properties(haulroute_cli PROPERTIES OUTPUT_NAME haulroute)
target_link_libraries(haulroute_cli PRIVATE haulroute)

enable_testing()
add_executable(haulroute_tests tests/test_main.cpp)
target_link_libraries(haulroute_tests PRIVATE haulroute)
add_test(NAME haulroute_tests COMMAND haulroute_tests)
