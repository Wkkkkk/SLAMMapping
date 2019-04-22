project(tools)

file(GLOB SOURCES_CXX *.cpp *.h)

find_package(Boost COMPONENTS log thread REQUIRED)

IF (UNIX)
    set(PTHREAD_LIB pthread)
ENDIF ()

add_library(${PROJECT_NAME} ${SOURCES_CXX})
target_link_libraries(${PROJECT_NAME} ${PTHREAD_LIB} ${Boost_LIBRARIES})