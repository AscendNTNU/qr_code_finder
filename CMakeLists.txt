cmake_minimum_required(VERSION 2.8)
project( FindQR )
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

file(GLOB SOURCES "src/*.cc")

add_executable( FindQR "src/main.cc" )
target_link_libraries( FindQR ${OpenCV_LIBS} )

