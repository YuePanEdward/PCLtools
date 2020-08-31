cmake_minimum_required(VERSION 2.8)

project(gsegpcdcut)

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

#libLas
FIND_PACKAGE(libLAS REQUIRED)
IF(libLAS_FOUND) 
MESSAGE("LibLAS [OK]") 
INCLUDE_DIRECTORIES(${LIBLAS_INCLUDE_DIR})
LINK_DIRECTORIES(${LIBLAS_LIBS})
LINK_LIBRARIES("/usr/local/lib/liblas.so.2.4.0")
ENDIF(libLAS_FOUND)

SET(DEP_LIBS ${DEP_LIBS} ${PCL_LIBRARIES} ${LibLAS_LIBRARIES})

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)

#add_executable(project2plane ${PROJECT_SOURCE_DIR}/test/project2plane.cpp)
#add_executable(cloud2raster ${PROJECT_SOURCE_DIR}/test/cloud2raster.cpp)
add_executable(gseg_pcdcut ${PROJECT_SOURCE_DIR}/test/gseg_pcdcut.cpp)

#target_link_libraries(project2plane ${DEP_LIBS})
#target_link_libraries(cloud2raster ${DEP_LIBS})
target_link_libraries(gseg_pcdcut ${DEP_LIBS})