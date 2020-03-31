cmake_minimum_required(VERSION 2.8.11)
project(myViewDemo)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_COMPILER "g++")
find_package(Qt4 COMPONENTS QtCore QtGui QtOpenGL REQUIRED)
include(${QT_USE_FILE})
include_directories(RL . rl_build/include)
add_executable(rlViewDemo rlViewDemo.cpp)
target_link_libraries(
	rlViewDemo
	${QT_QTCORE_LIBRARY}
	${QT_QTGUI_LIBRARY}
	${QT_QTOPENGL_LIBRARY}
	/usr/lib/x86_64-linux-gnu/libSoQt.so
	/home/derek/XR1_WS/src/src/RoboticsLib/rl_build/X86/librlhal.so
	/home/derek/XR1_WS/src/src/RoboticsLib/rl_build/X86/librlkin.so
	/home/derek/XR1_WS/src/src/RoboticsLib/rl_build/X86/librlmdl.so
	/home/derek/XR1_WS/src/src/RoboticsLib/rl_build/X86/librlplan.so
	/home/derek/XR1_WS/src/src/RoboticsLib/rl_build/X86/librlsg.so
)
