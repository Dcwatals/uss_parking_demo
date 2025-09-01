
cmake_minimum_required(VERSION 3.14)
project(uss_parking LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
find_package(Python3 COMPONENTS Development NumPy REQUIRED)
include_directories(${Python3_INCLUDE_DIRS})

add_library(uss STATIC
    src/USS.cpp
    # matplotlib-cpp-master/matplotlibcpp.h
)
# add_library(uss STATIC
#     # src/USS.cpp
#     matplotlib-cpp-master/matplotlibcpp.h
# )
link_libraries(${Python3_LIBRARIES})

target_include_directories(uss PUBLIC include)

add_executable(uss_demo
    src/main.cpp
)
target_link_libraries(uss_demo PRIVATE uss Python3::Python Python3::NumPy)
