cmake_minimum_required(VERSION 3.14)
project(project_template CXX)

set(CMAKE_CXX_STANDARD 20)

# for coverage analyzing
# if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#     set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fprofile-instr-generate -fcoverage-mapping")
# endif()

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

file(GLOB SRC_FILES "${CMAKE_SOURCE_DIR}/src/*.cpp")

add_executable(${PROJECT_NAME} ${CMAKE_SOURCE_DIR}/main.cpp ${SRC_FILES})

# add_library(Project_templateLib INTERFACE)
# target_include_directories(Project_templateLib INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)

# add_executable(${PROJECT_NAME} ${CMAKE_SOURCE_DIR}/main.cpp)
# target_link_libraries(${PROJECT_NAME} Project_templateLib)


# enable_testing()
# add_subdirectory(${CMAKE_SOURCE_DIR}/tests)

# set(CTEST_BINARY_DIRECTORY ${CMAKE_BINARY_DIR})
