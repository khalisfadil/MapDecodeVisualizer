# Prioritize libraries in /usr/local/lib
link_directories(/usr/local/lib)
set(CMAKE_BUILD_RPATH "/usr/local/lib")
set(CMAKE_INSTALL_RPATH "/usr/local/lib")

# Set CMake policies to suppress warnings
cmake_policy(SET CMP0167 NEW) # Modern FindBoost behavior
cmake_policy(SET CMP0072 NEW) # OpenGL handling
cmake_policy(SET CMP0144 NEW) # Modern package root variables

# Minimum CMake version required and project name
cmake_minimum_required(VERSION 3.10)
project(vizDynamicMap)

# Set the C++ standard and enforce it
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set build types (Release by default)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# set(CMAKE_BUILD_TYPE Debug)
# set(CMAKE_CXX_FLAGS_DEBUG "-g -Wall")

# Specify the compiler explicitly (if required)
set(CMAKE_C_COMPILER "/usr/bin/gcc-11")
set(CMAKE_CXX_COMPILER "/usr/bin/g++-11")

# Find required dependencies
find_package(Boost REQUIRED COMPONENTS system)
find_package(Open3D REQUIRED)
find_package(TBB REQUIRED)

# Manually add TBB and Eigen3 include paths if required
include_directories(/usr/include/tbb)
include_directories(/usr/include/eigen3)

# Define the core library
add_library(core STATIC
    source/callback/src/callbackStaticVoxel.cpp
    source/callback/src/callbackDynamicVoxel.cpp
    source/callback/src/callbackDynamicVoxelColor.cpp
    source/callback/src/callbackStaticVoxelColor.cpp
    source/callback/src/udpSocket.cpp
    source/viewer/src/top_down_viewer.cpp
)

# Add include directories for the core library
target_include_directories(core PUBLIC
    ${CMAKE_SOURCE_DIR}/source/callback/include
    ${CMAKE_SOURCE_DIR}/source/viewer/include
)

# Link necessary libraries to the core library
target_link_libraries(core
    Boost::system           # Boost system library for networking utilities
    Open3D::Open3D          # Open3D for 3D visualization and manipulation
    TBB::tbb                # Intel's Threading Building Blocks for multithreading
    pthread                 # POSIX threads
)

# Define the main application executable
add_executable(vizDynamicMap source/apps/vizDynamicMap.cpp)

# Add include directories for the application
target_include_directories(vizDynamicMap PRIVATE
    ${CMAKE_SOURCE_DIR}/source/callback/include
    ${CMAKE_SOURCE_DIR}/source/viewer/include
)

# Link the application executable to the core library
target_link_libraries(vizDynamicMap PRIVATE core)

# Debugging: Output configuration details for confirmation
message(STATUS "Project configuration:")
message(STATUS "  Boost Include: ${Boost_INCLUDE_DIRS}")
message(STATUS "  Eigen Include: /usr/include/eigen3")
message(STATUS "  Open3D Include: ${Open3D_INCLUDE_DIRS}")
message(STATUS "  TBB Include: /usr/include/tbb")
