project(messages)

find_package(Protobuf)
file(GLOB SOURCES_CXX *.cc *.cpp *.h)

add_library(${PROJECT_NAME} ${SOURCES_CXX})
target_link_libraries(${PROJECT_NAME} protobuf)