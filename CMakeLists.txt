cmake_minimum_required(VERSION 3.0)

project(SwingBall2D)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

if(UNIX)
  if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX "~" CACHE PATH "..." FORCE)
  endif()
endif()

add_executable(swingball2d main.cpp)

if(APPLE)
    message("-- Target is Apple")
    target_compile_options(stennis2d PUBLIC -DGL_SILENCE_DEPRECATION)
endif()

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

target_link_libraries(swingball2d ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} -lm)
target_include_directories(swingball2d PUBLIC ${GLUT_INCLUDE_DIR})

install(TARGETS swingball2d DESTINATION StangTennis2D)
