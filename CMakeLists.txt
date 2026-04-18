cmake_minimum_required(VERSION 3.12)
project(AsteroidMining)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(Easy3D COMPONENTS viewer REQUIRED)  # request Easy3D (recommended to request only needed components)
add_executable(main src/main.cpp)                # create an executable target
target_link_libraries(main easy3d::viewer)       # link to necessary Easy3D modules (add more if needed, e.g., algo)
