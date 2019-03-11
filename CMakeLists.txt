cmake_minimum_required(VERSION 3.5)
project(protype)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_PREFIX_PATH /usr/local/lib64:/usr/local/Qt-5.11.1/lib)

set(CXX_FLAGS
        -g
        -Wall
        -Wextra
        -Werror
#        -Wconversion
        -Wno-unused-parameter
        #        -Wold-style-cast
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

SET(MODULE_DIRS
        src/tools
        src/messages
        src/network
        src/client
        src/server
        )
FOREACH (MODULE_DIR ${MODULE_DIRS})
    include_directories(${MODULE_DIR})
    add_subdirectory(${MODULE_DIR})
ENDFOREACH (MODULE_DIR)

if(CMAKE_BUILD_TYPE MATCHES "Debug")
    message(STATUS "Building test......")
    add_subdirectory(src/test)
endif()