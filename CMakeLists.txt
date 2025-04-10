cmake_minimum_required(VERSION 3.14)
project(project_template CXX)

set(CMAKE_CXX_STANDARD 20)

# for coverage analyzing
# if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#     set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fprofile-instr-generate -fcoverage-mapping")
# endif()

# Try to find Eigen3 on the system first
find_package(Eigen3 QUIET)

# Include setup for directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# Define source files
file(GLOB SRC_FILES "${CMAKE_SOURCE_DIR}/src/*.cpp")

# Create executable
add_executable(${PROJECT_NAME} ${CMAKE_SOURCE_DIR}/main.cpp ${SRC_FILES})

if(NOT Eigen3_FOUND)
    message(STATUS "Eigen3 not found on system, downloading it...")

    # Path to download and extract Eigen
    set(EIGEN_VERSION 3.4.0)
    set(EIGEN_URL https://gitlab.com/libeigen/eigen/-/archive/${EIGEN_VERSION}/eigen-${EIGEN_VERSION}.tar.gz)
    set(EIGEN_DOWNLOAD_DIR "${CMAKE_CURRENT_BINARY_DIR}/eigen-download")
    set(EIGEN_INSTALL_DIR "${CMAKE_CURRENT_BINARY_DIR}/eigen-install")

    # Download Eigen
    include(ExternalProject)
    ExternalProject_Add(
        eigen
        URL ${EIGEN_URL}
        PREFIX ${EIGEN_DOWNLOAD_DIR}
        CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${EIGEN_INSTALL_DIR}
        INSTALL_COMMAND ""  # Skip install step
        UPDATE_COMMAND ""   # Skip update step
        CONFIGURE_COMMAND "" # Skip configure step
        BUILD_COMMAND ""    # Skip build step
    )

    # Set up include directory for Eigen
    set(EIGEN3_INCLUDE_DIR "${EIGEN_DOWNLOAD_DIR}/src/eigen")
    message(STATUS "Eigen3 include directory: ${EIGEN3_INCLUDE_DIR}")

    # Add dependency on the external project
    add_dependencies(${PROJECT_NAME} eigen)
else()
    message(STATUS "Found Eigen3: ${EIGEN3_INCLUDE_DIR}")
endif()

include_directories(${EIGEN3_INCLUDE_DIR})

# add_library(Project_templateLib INTERFACE)
# target_include_directories(Project_templateLib INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)

# add_executable(${PROJECT_NAME} ${CMAKE_SOURCE_DIR}/main.cpp)
# target_link_libraries(${PROJECT_NAME} Project_templateLib)


# enable_testing()
# add_subdirectory(${CMAKE_SOURCE_DIR}/tests)

# set(CTEST_BINARY_DIRECTORY ${CMAKE_BINARY_DIR})
