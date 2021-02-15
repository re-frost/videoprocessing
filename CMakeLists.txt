cmake_minimum_required(VERSION 3.5)

project(videoprocessing LANGUAGES CXX)
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


add_executable(videoprocessing main.cpp videoprocessing.cpp show.cpp frameprocessing.cpp staticUtils.h)
target_link_libraries( videoprocessing ${OpenCV_LIBS} )
