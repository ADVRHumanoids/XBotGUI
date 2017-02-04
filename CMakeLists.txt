cmake_minimum_required(VERSION 2.8.3)
project(pilot_interface)

add_definitions(-std=c++11)

set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_BINARY_DIR})

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
find_package(XBotCoreModel REQUIRED)

include(${QT_USE_FILE})
include_directories(include ${XBotCoreModel_INCLUDE_DIRS})

add_definitions(-DQT_NO_KEYWORDS)

qt4_wrap_cpp(XBOT_MOC_FILES
  include/XBotGUI/main_interface.h
)

file(GLOB LOW_LEVEL_CONTROL_H_FILES
    "${CMAKE_SOURCE_DIR}/include/XBotGUI/low_level_control/*.h"
)

qt4_wrap_cpp(LOW_LEVEL_CONTROL_MOC_FILES
  ${LOW_LEVEL_CONTROL_H_FILES}
)

file(GLOB LOW_LEVEL_CONTROL_SOURCE_FILES
    "${CMAKE_SOURCE_DIR}/src/low_level_control/*.cpp"
)

set(XBOT_SOURCE_FILES
  ${XBOT_MOC_FILES}
  ${LOW_LEVEL_CONTROL_MOC_FILES}
  ${LOW_LEVEL_CONTROL_SOURCE_FILES}
  src/main_interface.cpp
  src/utils.cpp
)

add_executable(main_interface src/main.cpp ${XBOT_SOURCE_FILES})

target_link_libraries(main_interface ${QT_LIBRARIES} yaml-cpp ${XBotCoreModel_LIBRARIES})

install(DIRECTORY resources/
    DESTINATION "resources"
    FILES_MATCHING PATTERN "*.png")