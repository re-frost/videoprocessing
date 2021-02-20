cmake_minimum_required(VERSION 3.5)

project(videoprocessing LANGUAGES CXX)
find_package( OpenCV REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS} include/)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


add_executable(videoprocessing
    src/main.cpp
    src/videoprocessing.cpp
    src/show.cpp
    src/frameprocessing.cpp
    include/staticUtils.hpp
    src/colortracking.cpp)

target_link_libraries( videoprocessing ${OpenCV_LIBS} )
