cmake_minimum_required(VERSION 2.8)

project(project2plane)

SET(CMAKE_CXX_FLAGS "-O3 -std=c++11 -w -fopenmp -pthread")
SET(CMAKE_C_FLAGS "-O3 -std=c++11 -fopenmp -pthread")

#SET(CMAKE_BUILD_TYPE "Debug") 
#SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")


SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

#PCL
FIND_PACKAGE(PCL REQUIRED)
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
ADD_DEFINITIONS(${PCL_DEFINITIONS})
LIST(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
message("PCL [OK]")

SET(DEP_LIBS ${DEP_LIBS} ${PCL_LIBRARIES})

add_executable(project2plane ${PROJECT_SOURCE_DIR}/src/main.cpp)

target_link_libraries(project2plane ${DEP_LIBS})