cmake_minimum_required(VERSION 3.12)
project(asteroid_mining)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

find_package(Boost REQUIRED)
find_package(Easy3D REQUIRED COMPONENTS algo core renderer util viewer)
find_package(Eigen3 REQUIRED)

include_directories(include ${Easy3D_INCLUDE_DIR})

add_library(${PROJECT_NAME}_math SHARED src/math.cpp)
target_link_libraries(${PROJECT_NAME}_math
    Eigen3::Eigen
    Boost::boost
)

add_library(${PROJECT_NAME}_dimensions_scaler SHARED src/dimensions_scaler.cpp)

add_library(${PROJECT_NAME}_asteroid SHARED src/asteroid.cpp)
target_link_libraries(${PROJECT_NAME}_asteroid
    ${PROJECT_NAME}_math
    ${PROJECT_NAME}_dimensions_scaler
)

add_library(${PROJECT_NAME}_siphon SHARED src/siphon.cpp)
target_link_libraries(${PROJECT_NAME}_siphon
    ${PROJECT_NAME}_asteroid
)

add_library(${PROJECT_NAME}_released_payload SHARED src/released_payload.cpp)
target_link_libraries(${PROJECT_NAME}_released_payload
    ${PROJECT_NAME}_asteroid
)

add_library(${PROJECT_NAME}_model SHARED src/model.cpp)
target_link_libraries(${PROJECT_NAME}_model
    ${PROJECT_NAME}_siphon
    ${PROJECT_NAME}_released_payload
)

add_library(${PROJECT_NAME}_view SHARED src/view.cpp)
target_link_libraries(${PROJECT_NAME}_view
    easy3d::algo
    easy3d::renderer
    easy3d::viewer
    ${PROJECT_NAME}_model
)

add_executable(${PROJECT_NAME}_main src/main.cpp)
target_link_libraries(${PROJECT_NAME}_main
    ${PROJECT_NAME}_view
    easy3d::util
)

install(TARGETS
    ${PROJECT_NAME}_math
    ${PROJECT_NAME}_dimensions_scaler
    ${PROJECT_NAME}_asteroid
    ${PROJECT_NAME}_siphon
    ${PROJECT_NAME}_released_payload
    ${PROJECT_NAME}_model
    ${PROJECT_NAME}_view
    EXPORT export_${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)
install(TARGETS ${PROJECT_NAME}_main
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)
