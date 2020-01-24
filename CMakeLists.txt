cmake_minimum_required (VERSION 3.0.0)
 
project (PCL-DBSCAN)
set(EXECUTABLE_OUTPUT_PATH ./${CMAKE_BUILD_TYPE})

# specify the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_compile_options(-std=c++11)


find_package(Boost COMPONENTS filesystem system REQUIRED)

include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

find_package(PCL 1.8 REQUIRED COMPONENTS common io visualization)

include_directories(${PCL_INCLUDE_DIRS})
link_directories( ${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(inc)

set(SOURCES
    src/demo
    src/DBSCANFilter.cpp)

add_executable (demo ${SOURCES})

target_link_libraries(demo ${Boost_LIBRARIES} ${PCL_LIBRARIES})
