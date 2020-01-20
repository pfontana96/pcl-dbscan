cmake_minimum_required (VERSION 3.0.0)
 
project (PCL-DBSCAN)
set(EXECUTABLE_OUTPUT_PATH ./${CMAKE_BUILD_TYPE})

# specify the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

include_directories(inc)

file(
    GLOB_RECURSE
    MY_SOURCE_FILES
    src/*
    inc/*
)

add_executable (${PROJECT_NAME} ${MY_SOURCE_FILES})

target_include_directories(${PROJECT_NAME} PUBLIC
                            "${PROJECT_BINARY_DIR}"
                            ${DBSCAN_INCLUDES})
