project(network)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

file(GLOB SOURCES_CXX *.cpp *.h)
add_library(${PROJECT_NAME} ${SOURCES_CXX})

target_link_libraries(${PROJECT_NAME} messages libmuduo_base.a libmuduo_net.a pthread rt)