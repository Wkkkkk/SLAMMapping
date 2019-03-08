cmake_minimum_required(VERSION 3.5)
project(tools)

file(GLOB SOURCES_CXX *.cpp *.h)

add_library(${PROJECT_NAME} ${SOURCES_CXX})
target_link_libraries(${PROJECT_NAME} pthread)