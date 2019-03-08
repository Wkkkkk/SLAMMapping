cmake_minimum_required(VERSION 3.5)
project(test)

file(GLOB SOURCES_CXX "*.cpp")

add_executable(${PROJECT_NAME} ${SOURCES_CXX})
target_link_libraries(${PROJECT_NAME} tools)