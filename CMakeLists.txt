CMAKE_MINIMUM_REQUIRED(VERSION 2.8.11)

SET(PROJECTNAME RBOT)
PROJECT(${PROJECTNAME})

SET(CMAKE_VERBOSE_MAKEFILE ON)
SET(OpenCV_STATIC OFF)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}
		"${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")

SET(CMAKE_INCLUDE_CURRENT_DIR ON)
SET(CMAKE_AUTOMOC ON)

# 收集所有通用源文件（不包含 main 函数的）
FILE(GLOB SRC_COMMON src/*.cpp src/*.h src/*.hpp src/*.glsl)

# 排除各个主程序入口
list(REMOVE_ITEM SRC_COMMON
		${CMAKE_CURRENT_SOURCE_DIR}/src/test.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/src/test2.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/src/compare_obj_ply.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/src/calibration.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/src/cube_tracking.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
)

# 查找依赖
FIND_PACKAGE(OpenGL REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(Qt5Widgets REQUIRED)
FIND_PACKAGE(Qt5OpenGL REQUIRED)
FIND_PACKAGE(ASSIMP REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)

# 头文件路径
INCLUDE_DIRECTORIES(
		${OpenCV_INCLUDE_DIRS}
		${OPENGL_INCLUDE_DIR}
		${ASSIMP_INCLUDE_DIR}
		${EIGEN3_INCLUDE_DIRS}
)

# 通用依赖库
SET(LIBRARIES
		Qt5::Widgets
		Qt5::OpenGL
		${OpenCV_LIBS}
		${OPENGL_LIBRARIES}
		${ASSIMP_LIBRARIES}
)

# 定义多个可执行文件
add_executable(test         ${SRC_COMMON} src/test.cpp)
add_executable(test2        ${SRC_COMMON} src/test2.cpp)
add_executable(compare_obj  ${SRC_COMMON} src/compare_obj_ply.cpp)
add_executable(calibration  ${SRC_COMMON} src/calibration.cpp)
add_executable(cube_track   ${SRC_COMMON} src/cube_tracking.cpp)
add_executable(main   ${SRC_COMMON} src/main.cpp)


# 链接库
target_link_libraries(test         ${LIBRARIES})
target_link_libraries(test2        ${LIBRARIES})
target_link_libraries(compare_obj  ${LIBRARIES})
target_link_libraries(calibration  ${LIBRARIES})
target_link_libraries(cube_track   ${LIBRARIES})
target_link_libraries(main   ${LIBRARIES})

# 输出路径
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
