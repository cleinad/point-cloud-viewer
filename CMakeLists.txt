cmake_minimum_required(VERSION 3.10)

# Declare project name
project(PointCloudViewer LANGUAGES C CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find PCL library, including 'visualization' component
find_package(PCL 1.10 REQUIRED COMPONENTS visualization)

if(PCL_FOUND)
    # Define executable
    add_executable(pcv
        src/main.cpp
        src/PointCloud.cpp
    )

    # Include PCL headers
    target_include_directories(pcv PRIVATE ${PCL_INCLUDE_DIRS})

    # Link against PCL only
    target_link_libraries(pcv
        PRIVATE
            ${PCL_LIBRARIES}
    )
endif()

find_package(MPI REQUIRED COMPONENTS C CXX)
message(STATUS "MPI_FOUND: ${MPI_FOUND}")
message(STATUS "MPI_C_FOUND: ${MPI_C_FOUND}")
message(STATUS "MPI_CXX_FOUND: ${MPI_CXX_FOUND}")