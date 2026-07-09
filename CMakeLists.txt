cmake_minimum_required(VERSION 3.14)
project(RBDEngine VERSION 1.0 LANGUAGES CXX)

# Require C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set up the main executable
add_executable(rbd_sim src/main.cpp)

# Direct Include Paths - Now pointing one level deeper!
target_include_directories(rbd_sim PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    "C:/Users/91953/Desktop/Libraries/eigen-5.0.0/eigen-5.0.0"
)