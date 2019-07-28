cmake_minimum_required(VERSION 3.14)
project(robot_write)

#set(CMAKE_CXX_STANDARD 14)

# 自动调用moc，uic，rcc处理qt的扩展部分
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build/bin
        CACHE PATH "Runtime directory" FORCE)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/qt
        ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/opencv-3.4.1
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/qt
        ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/opencv-3.4.1
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# 引入kinect2相关的依赖
set(FREENECT_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/denpendents/include)
set(FREENECT_LIBRARIES ${PROJECT_SOURCE_DIR}/denpendents/lib)

include_directories(${FREENECT_INCLUDE_DIRS})
link_directories(${FREENECT_LIBRARIES})

# 引入libtorch。
set(Torch_DIR ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/libtorch/share/cmake/Torch)
find_package(Torch REQUIRED)

# 引入Aubo机器人驱动。
find_package(Aubo REQUIRED)

find_package(OpenCV REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5OpenGL REQUIRED)
find_package(Qt5Network REQUIRED)
find_package(Qt5Xml REQUIRED)
find_package(Qt5Qml REQUIRED)
find_package(PCL REQUIRED)

# 引入Aubo机器人驱动。
include_directories(${Aubo_INCLUDE_DIRS})
message(STATUS "add_Aubo_include:" ${INCLUDE_DIRS})

# 引入libtorch
include_directories(${TORCH_INCLUDE_DIRS})

include_directories(${OpenCV_INCLUDE_DIRS} ${Qt5Core_INCLUDE_DIRS} ${Qt5Gui_INCLUDE_DIRS}
        ${Qt5Widgets_INCLUDE_DIRS} ${Qt5OpenGL_INCLUDE_DIRS} ${Qt5Network_INCLUDE_DIRS}
        ${Qt5Xml_INCLUDE_DIRS} ${Qt5Qml_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
include_directories("/usr/include/eigen3")
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_subdirectory(src)
