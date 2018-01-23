cmake_minimum_required(VERSION 2.8.3)
project(XBotGUI)

add_definitions(-std=c++11)

option(USING_ROS "Compile many ROS based stuff" ON)
option(USE_ROBOT_SLIDERS "Add sliders to command the robot joints in the GUI" ON)
option(USING_COMMA_AS_DECIMAL_SEPARATOR "parsing string with atof or stod works with dots or commas, depending on the system" OFF)
configure_file( cmake_options.h.in ${CMAKE_BINARY_DIR}/generated/cmake_options.h )
include_directories( ${CMAKE_BINARY_DIR}/generated/ )

#set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_BINARY_DIR})

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(XBotCoreModel REQUIRED)
find_package(XBotInterface REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)

include_directories(include ${XBotCoreModel_INCLUDE_DIRS} ${XBotInterface_INCLUDE_DIRS} ${Qt5Widgets_INCLUDE_DIRS} ${Qt5Gui_INCLUDE_DIRS})

file(GLOB LOW_LEVEL_CONTROL_H_FILES
    "${CMAKE_SOURCE_DIR}/include/XBotGUI/low_level_control/*.h"
)

file(GLOB LOW_LEVEL_CONTROL_SOURCE_FILES
    "${CMAKE_SOURCE_DIR}/src/low_level_control/*.cpp"
)

if($ENV{ROS_DISTRO} STRLESS "indigo" OR $ENV{ROS_DISTRO} STREQUAL "indigo")
    find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
    include(${QT_USE_FILE})
    # wrap using Qt
        
    qt4_wrap_cpp(XBOT_MOC_FILES include/XBotGUI/main_interface.h)
    qt4_wrap_cpp(LOW_LEVEL_CONTROL_MOC_FILES ${LOW_LEVEL_CONTROL_H_FILES})
    qt4_wrap_cpp(UTILS_MOC_FILES ${UTILS_H_FILES})
    
    if(USING_ROS) #if so, required variables are filled
        find_package(catkin REQUIRED COMPONENTS rviz roscpp roslib rospy tf_conversions)
        include_directories(${catkin_INCLUDE_DIRS})
        file(GLOB INTERACTION_H_FILES
                "${CMAKE_SOURCE_DIR}/include/XBotGUI/interaction/*.h"
        )
        file(GLOB INTERACTION_SOURCE_FILES
                "${CMAKE_SOURCE_DIR}/src/interaction/*.cpp"
        )
        qt4_wrap_cpp(INTERACTION_MOC_FILES ${INTERACTION_H_FILES})
        file(GLOB UTILS_H_FILES
            "${CMAKE_SOURCE_DIR}/include/XBotGUI/utils/*.h"
        )
        file(GLOB UTILS_SOURCE_FILES
            "${CMAKE_SOURCE_DIR}/src/utils/*.cpp"
        )
        qt4_wrap_cpp(UTILS_MOC_FILES ${UTILS_H_FILES})
    endif()


elseif($ENV{ROS_DISTRO} STRGREATER "indigo")
    find_package(Qt5 COMPONENTS Core Widgets Gui REQUIRED)
    include_directories(${Qt5Core_INCLUDE_DIRS} ${Qt5Widgets_INCLUDE_DIRS})
    add_definitions(-DQT_NO_KEYWORDS)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}")
    
    # wrap using Qt
    
    qt5_wrap_cpp(XBOT_MOC_FILES include/XBotGUI/main_interface.h)
    qt5_wrap_cpp(LOW_LEVEL_CONTROL_MOC_FILES ${LOW_LEVEL_CONTROL_H_FILES})
    qt5_wrap_cpp(UTILS_MOC_FILES ${UTILS_H_FILES})

    if(USING_ROS) #if so, required variables are filled
        find_package(catkin REQUIRED COMPONENTS rviz roscpp roslib rospy tf_conversions)
        include_directories(${catkin_INCLUDE_DIRS})
        file(GLOB INTERACTION_H_FILES
                "${CMAKE_SOURCE_DIR}/include/XBotGUI/interaction/*.h"
        )
        file(GLOB INTERACTION_SOURCE_FILES
                "${CMAKE_SOURCE_DIR}/src/interaction/*.cpp"
        )
        qt5_wrap_cpp(INTERACTION_MOC_FILES ${INTERACTION_H_FILES})
        file(GLOB UTILS_H_FILES
            "${CMAKE_SOURCE_DIR}/include/XBotGUI/utils/*.h"
        )
        file(GLOB UTILS_SOURCE_FILES
            "${CMAKE_SOURCE_DIR}/src/utils/*.cpp"
        )
        qt5_wrap_cpp(UTILS_MOC_FILES ${UTILS_H_FILES})
    endif()


    SET(QT_LIBRARIES
        ${Qt5Core_LIBRARIES}
        ${Qt5Widgets_LIBRARIES}
        ${Qt5Gui_LIBRARIES}
    )
endif()




set(XBOT_SOURCE_FILES
  ${XBOT_MOC_FILES}
  ${LOW_LEVEL_CONTROL_MOC_FILES}
  ${LOW_LEVEL_CONTROL_SOURCE_FILES}
  ${INTERACTION_MOC_FILES}
  ${INTERACTION_SOURCE_FILES}
  ${UTILS_MOC_FILES}
  ${UTILS_SOURCE_FILES}
  src/main_interface.cpp
  src/print_utils.cpp
)

add_executable(XBotGUI src/main.cpp ${XBOT_SOURCE_FILES})

if($ENV{ROS_DISTRO} STRLESS "indigo" OR $ENV{ROS_DISTRO} STREQUAL "indigo")
#     qt4_use_modules(XBotGUI )
elseif($ENV{ROS_DISTRO} STRGREATER "indigo")
    qt5_use_modules(XBotGUI Widgets Gui)
endif()

target_link_libraries(XBotGUI ${QT_LIBRARIES} yaml-cpp ${XBotCoreModel_LIBRARIES} ${XBotInterface_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS XBotGUI
        EXPORT  XBotGUI
        RUNTIME DESTINATION bin)

install(DIRECTORY resources/
    DESTINATION "resources"
    FILES_MATCHING PATTERN "*.png")
    
install(DIRECTORY configs/
    DESTINATION "configs"
    FILES_MATCHING PATTERN "*.xml")
