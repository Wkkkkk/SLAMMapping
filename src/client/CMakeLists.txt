set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

find_package(Qt5LinguistTools)
find_package(Qt5Core 5.11 REQUIRED)
find_package(Qt5Widgets 5.11 REQUIRED)
find_package(Qt5Core 5.11 REQUIRED)
find_package(Qt5Gui 5.11 REQUIRED)
find_package(Qt5Network 5.11 REQUIRED)

include_directories(render)
include_directories(ui)

add_subdirectory(render)
add_subdirectory(ui)