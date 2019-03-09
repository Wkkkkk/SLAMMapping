project(server)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

file(GLOB SOURCES_CXX "*.cpp")

add_executable(${PROJECT_NAME} ${SOURCES_CXX})
target_link_libraries(${PROJECT_NAME} network tools)