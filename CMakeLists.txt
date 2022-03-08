cmake_minimum_required(VERSION 3.22)

project( eigen-test )

file( GLOB_RECURSE headers Eigen/* )
set( sources main.cpp )

message( "${CMAKE_SOURCE_DIR}" )

add_executable( main ${sources} ${headers} )
target_include_directories( main PRIVATE ${CMAKE_SOURCE_DIR} )
