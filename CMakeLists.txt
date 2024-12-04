# Set minimum CMake version and project name
cmake_minimum_required(VERSION 3.16)
project(vizDynamicMap)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set build types
set(CMAKE_BUILD_TYPE Release)  # Default to Release
set(CMAKE_CXX_FLAGS_DEBUG "-g -fsanitize=thread,address")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# Library priorities
link_directories(/usr/local/lib)
set(CMAKE_BUILD_RPATH "$ORIGIN")
set(CMAKE_INSTALL_RPATH "$ORIGIN")

# CMake policies
cmake_policy(SET CMP0072 NEW) # OpenGL handling

# Find required dependencies
set(Open3D_DIR "/usr/local/lib/cmake/Open3D/") # Ensure correct directory
set(TBB_DIR "/usr/lib/x86_64-linux-gnu/cmake/TBB") # Ensure correct directory
find_package(Open3D REQUIRED)
find_package(Boost 1.83 REQUIRED COMPONENTS system)
find_package(TBB REQUIRED)
find_package(Eigen3 3.4 REQUIRED NO_MODULE)
find_package(PCL 1.14 REQUIRED)
find_package(OpenMP REQUIRED) # Include OpenMP

# Debugging: Output found paths for PCL
message(STATUS "PCL Include Dirs: ${PCL_INCLUDE_DIRS}")
message(STATUS "PCL Libraries: ${PCL_LIBRARIES}")

# Core library
add_library(core STATIC
    source/callback/src/callbackStaticVoxel.cpp
    source/callback/src/callbackDynamicVoxel.cpp
    source/callback/src/callbackDynamicVoxelColor.cpp
    source/callback/src/callbackStaticVoxelColor.cpp
    source/callback/src/udpSocket.cpp
    source/callback/src/callbackPoints.cpp
    source/viewer/src/top_down_viewer.cpp
    source/3rdparty/dynamic-map/source/core/src/clusterExtractor.cpp
    source/3rdparty/dynamic-map/source/core/src/EKFVelocity2D.cpp
    source/3rdparty/dynamic-map/source/core/src/occupancyMap.cpp
)

# Include directories for the core library
target_include_directories(core PUBLIC
    ${CMAKE_SOURCE_DIR}/source/3rdparty/robin-map/include/tsl
    ${CMAKE_SOURCE_DIR}/source/3rdparty/dynamic-map/source/core/include
    ${CMAKE_SOURCE_DIR}/source/callback/include
    ${CMAKE_SOURCE_DIR}/source/viewer/include
    ${PCL_INCLUDE_DIRS} # PCL include directories
)

# Link necessary libraries to the core library
target_link_libraries(core
    Boost::system         # Boost system library for networking utilities
    Open3D::Open3D        # Open3D for 3D visualization and manipulation
    TBB::tbb              # Intel's Threading Building Blocks for multithreading
    Eigen3::Eigen         # Eigen library
    ${PCL_LIBRARIES}      # Link PCL libraries
    OpenMP::OpenMP_CXX    # OpenMP for parallel processing
)

# Define the main application executables
add_executable(vizDynamicMap source/apps/vizDynamicMap.cpp)
add_executable(vizDynamicMap2 source/apps/vizDynamicMap2.cpp)
add_executable(vizPoints source/apps/vizPoints.cpp)

# Include directories for the applications
target_include_directories(vizDynamicMap PRIVATE
    ${CMAKE_SOURCE_DIR}/source/3rdparty/robin-map/include/tsl
    ${CMAKE_SOURCE_DIR}/source/3rdparty/dynamic-map/source/core/include
    ${CMAKE_SOURCE_DIR}/source/callback/include
    ${CMAKE_SOURCE_DIR}/source/viewer/include
    ${PCL_INCLUDE_DIRS} # PCL include directories
)

target_include_directories(vizDynamicMap2 PRIVATE
    ${CMAKE_SOURCE_DIR}/source/3rdparty/robin-map/include/tsl
    ${CMAKE_SOURCE_DIR}/source/3rdparty/dynamic-map/source/core/include
    ${CMAKE_SOURCE_DIR}/source/callback/include
    ${CMAKE_SOURCE_DIR}/source/viewer/include
    ${PCL_INCLUDE_DIRS} # PCL include directories
)

target_include_directories(vizPoints PRIVATE
    ${CMAKE_SOURCE_DIR}/source/3rdparty/robin-map/include/tsl
    ${CMAKE_SOURCE_DIR}/source/3rdparty/dynamic-map/source/core/include
    ${CMAKE_SOURCE_DIR}/source/callback/include
    ${CMAKE_SOURCE_DIR}/source/viewer/include
    ${PCL_INCLUDE_DIRS} # PCL include directories
)

# Link the application executables to the core library
target_link_libraries(vizDynamicMap PRIVATE core)
target_link_libraries(vizDynamicMap2 PRIVATE core)
target_link_libraries(vizPoints PRIVATE core)

# Debugging: Output configuration details for confirmation
message(STATUS "Project configuration:")
message(STATUS "  Boost Version: ${Boost_VERSION}")
message(STATUS "  Open3D Version: 0.18.0")
message(STATUS "  TBB_DIR set to: ${TBB_DIR}")
message(STATUS "  Eigen Version: 3.4.0")
message(STATUS "  PCL Version: 1.14")
message(STATUS "  OpenMP Found: ${OpenMP_FOUND}")
message(STATUS "  Compiler: ${CMAKE_CXX_COMPILER}")
message(STATUS "  Compiler Version: ${CMAKE_CXX_COMPILER_VERSION}")
