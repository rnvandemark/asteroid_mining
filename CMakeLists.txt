cmake_minimum_required(VERSION 3.12)
project(asteroid_mining)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Boost REQUIRED)
find_package(Easy3D REQUIRED COMPONENTS algo core renderer util viewer)
find_package(Eigen3 REQUIRED)

include_directories(include)

###############################################################################
# Generate libraries
###############################################################################

add_library(${PROJECT_NAME}_math SHARED src/math.cpp)
target_link_libraries(${PROJECT_NAME}_math PUBLIC
    Eigen3::Eigen
    Boost::boost
)
target_include_directories(${PROJECT_NAME}_math PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_library(${PROJECT_NAME}_dimensions_scaler SHARED src/dimensions_scaler.cpp)
target_include_directories(${PROJECT_NAME}_dimensions_scaler PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_library(${PROJECT_NAME}_carriers SHARED
    src/asteroid/carrier.cpp
    src/released_payload/carrier.cpp
    src/siphon/carrier.cpp
)
target_link_libraries(${PROJECT_NAME}_carriers PUBLIC Eigen3::Eigen)
target_include_directories(${PROJECT_NAME}_carriers PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_library(${PROJECT_NAME}_mementos SHARED
    src/asteroid/memento.cpp
    src/released_payload/memento.cpp
    src/siphon/memento.cpp
)
target_link_libraries(${PROJECT_NAME}_mementos PUBLIC ${PROJECT_NAME}_carriers)
target_include_directories(${PROJECT_NAME}_mementos PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_library(${PROJECT_NAME}_originators SHARED
    src/asteroid/originator.cpp
    src/released_payload/originator.cpp
    src/siphon/originator.cpp
)
target_link_libraries(${PROJECT_NAME}_originators PUBLIC
    ${PROJECT_NAME}_carriers
    ${PROJECT_NAME}_math
    ${PROJECT_NAME}_dimensions_scaler
)
target_include_directories(${PROJECT_NAME}_originators PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_library(${PROJECT_NAME}_model SHARED src/model.cpp)
target_link_libraries(${PROJECT_NAME}_model PUBLIC ${PROJECT_NAME}_originators)
target_include_directories(${PROJECT_NAME}_model PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_library(${PROJECT_NAME}_view SHARED src/view.cpp)
target_link_libraries(${PROJECT_NAME}_view PUBLIC
    easy3d::algo
    easy3d::renderer
    easy3d::viewer
    ${PROJECT_NAME}_model
)
target_include_directories(${PROJECT_NAME}_view PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

###############################################################################
# Build executable
###############################################################################

add_executable(${PROJECT_NAME}_main src/main.cpp)
target_link_libraries(${PROJECT_NAME}_main PUBLIC
    ${PROJECT_NAME}_view
    easy3d::util
)
target_include_directories(${PROJECT_NAME}_main PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

###############################################################################
# Install everything
###############################################################################

install(TARGETS
    ${PROJECT_NAME}_math
    ${PROJECT_NAME}_dimensions_scaler
    ${PROJECT_NAME}_carriers
    ${PROJECT_NAME}_mementos
    ${PROJECT_NAME}_originators
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
install(
    DIRECTORY include/
    DESTINATION include
)

###############################################################################
# Generate CMake files for downstream packages
###############################################################################

include(CMakePackageConfigHelpers)
install(
    EXPORT export_${PROJECT_NAME}
    FILE "${PROJECT_NAME}Targets.cmake"
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION lib/cmake/${PROJECT_NAME}
)
configure_package_config_file(
    "${PROJECT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in"
    "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
)
install(FILES
    "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    DESTINATION lib/cmake/${PROJECT_NAME}
)
