cmake_minimum_required(VERSION 3.10)
project(Planning)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED true)

# Find and configure OpenCV
find_package(OpenCV REQUIRED)
find_package(SFML 2.5 COMPONENTS graphics audio REQUIRED)

# Add include directories for OpenCV headers
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(include)
# Add your project's source files
add_executable(Planning src/main.cpp 
            src/d_star_lite.cpp
            src/nodes.cpp 
            src/global_planner.cpp)

# Link OpenCV libraries to your project
target_link_libraries(Planning ${OpenCV_LIBS})
target_link_libraries(Planning  sfml-graphics  sfml-system)
