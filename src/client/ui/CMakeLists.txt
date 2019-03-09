project(client)

file(GLOB SOURCES_CXX *.cpp *.ui *.h *.qrc)

add_executable(${PROJECT_NAME} ${SOURCES_CXX})
target_link_libraries(${PROJECT_NAME} network render)
qt5_use_modules(${PROJECT_NAME} Sql Gui Widgets OpenGL Network Concurrent)