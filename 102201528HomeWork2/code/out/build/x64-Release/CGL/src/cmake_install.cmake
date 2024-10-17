# Install script for directory: E:/Work/102201528HomeWork2/code/CGL/src

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "E:/Work/102201528HomeWork2/code/out/install/x64-Release")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "RelWithDebInfo")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "E:/Work/102201528HomeWork2/code/out/build/x64-Release/CGL/src/CGL.lib")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("E:/Work/102201528HomeWork2/code/out/build/x64-Release/CGL/src/CMakeFiles/CGL.dir/install-cxx-module-bmi-RelWithDebInfo.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/CGL" TYPE FILE FILES
    "E:/Work/102201528HomeWork2/code/CGL/src/CGL.h"
    "E:/Work/102201528HomeWork2/code/CGL/src/vector2D.h"
    "E:/Work/102201528HomeWork2/code/CGL/src/vector3D.h"
    "E:/Work/102201528HomeWork2/code/CGL/src/vector4D.h"
    "E:/Work/102201528HomeWork2/code/CGL/src/matrix3x3.h"
    "E:/Work/102201528HomeWork2/code/CGL/src/matrix4x4.h"
    "E:/Work/102201528HomeWork2/code/CGL/src/quaternion.h"
    "E:/Work/102201528HomeWork2/code/CGL/src/complex.h"
    "E:/Work/102201528HomeWork2/code/CGL/src/color.h"
    "E:/Work/102201528HomeWork2/code/CGL/src/osdtext.h"
    "E:/Work/102201528HomeWork2/code/CGL/src/viewer.h"
    "E:/Work/102201528HomeWork2/code/CGL/src/base64.h"
    "E:/Work/102201528HomeWork2/code/CGL/src/tinyxml2.h"
    "E:/Work/102201528HomeWork2/code/CGL/src/renderer.h"
    )
endif()

