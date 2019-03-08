cmake_minimum_required(VERSION 3.5)
project(protype)
set(CMAKE_CXX_STANDARD 14)

set(CXX_FLAGS
        -g
        -Wall
        -Wextra
        -Werror
#        -Wconversion
        -Wno-unused-parameter
        -Wold-style-cast
        -Woverloaded-virtual
        -Wpointer-arith
        -Wshadow
        -Wwrite-strings
        -march=native
        -rdynamic
        )
string(REPLACE ";" " " CMAKE_CXX_FLAGS "${CXX_FLAGS}")

set(CMAKE_CXX_FLAGS_DEBUG "-O0")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -finline-limit=1000 -DNDEBUG")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_subdirectory(src/tools)
add_subdirectory(src/network)
add_subdirectory(src/messages)

add_subdirectory(src/client)
add_subdirectory(src/server)

if(CMAKE_BUILD_TYPE MATCHES "Debug")
    message(STATUS "Building test......")
    add_subdirectory(src/test)
endif()