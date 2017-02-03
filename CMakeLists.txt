cmake_minimum_required(VERSION 2.8.3)
project(pilot_interface)

add_definitions(-std=c++11)

set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_BINARY_DIR})

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)

include(${QT_USE_FILE})
include_directories(include)

add_definitions(-DQT_NO_KEYWORDS)

qt4_wrap_cpp(XBOT_MOC_FILES
  include/XBotGUI/main_interface.h
)

set(XBOT_SOURCE_FILES
  ${XBOT_MOC_FILES}
  src/main_interface.cpp
  src/utils.cpp
)

add_executable(main_interface src/main.cpp ${XBOT_SOURCE_FILES})

target_link_libraries(main_interface ${QT_LIBRARIES} yaml-cpp)

install(DIRECTORY resources/
    DESTINATION "resources"
    FILES_MATCHING PATTERN "*.png")