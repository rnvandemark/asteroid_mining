cmake_minimum_required(VERSION 3.12)
project(AsteroidMining)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Easy3D REQUIRED COMPONENTS viewer)
find_package(Boost REQUIRED)

add_executable(main src/main.cpp)
target_link_libraries(main Boost::boost easy3d::viewer)
