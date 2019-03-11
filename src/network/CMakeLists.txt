project(network)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(Boost REQUIRED)
find_package(PCL REQUIRED)
find_package(Draco REQUIRED)

include_directories(
        ${Boost_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)

file(GLOB SOURCES_CXX *.cpp *.h)
add_library(${PROJECT_NAME} ${SOURCES_CXX})

target_link_libraries(${PROJECT_NAME}
        messages
        pthread rt
        libmuduo_base.a libmuduo_net.a
        draco dracoenc dracodec
        ${PCL_LIBRARIES}
        )