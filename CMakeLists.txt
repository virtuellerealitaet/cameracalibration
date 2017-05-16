cmake_minimum_required(VERSION 3.6)
project(cameracalibration)


# ========================================================================================= OPTIONS

set(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "" FORCE)

# ======================================================================================= LIBRARIES
find_package(OpenGL REQUIRED)

set(lib_dir ${CMAKE_SOURCE_DIR}/lib)

if (UNIX)
  find_package(OpenCV REQUIRED)
  set(OpenCV_LIBS_DEBUG ${OpenCV_LIBS})
  find_package(libusb REQUIRED)
  
else()
  if (OpenCV_BASE_PATH)
    set(OpenCV_LIB_DEBUG ${OpenCV_BASE_PATH}/lib/opencv_world320d.lib CACHE FILEPATH "")
    set(OpenCV_LIB_RELEASE ${OpenCV_BASE_PATH}/lib/opencv_world320.lib CACHE FILEPATH "")
    set(OpenCV_LIBS debug ${OpenCV_LIB_DEBUG} optimized ${OpenCV_LIB_RELEASE})
    set(OpenCV_INCLUDES ${OpenCV_BASE_PATH}/include/ CACHE PATH "")
  endif()
  set(OpenCV_BASE_PATH $ENV{OPENCV_310} CACHE PATH "")
  
  
    set(LIBUSB_LIB_DEBUG ../lib/libusbd CACHE FILEPATH "")
    set(LIBUSB_LIB_RELEASE ../lib/libusb CACHE FILEPATH "")
    set(LIBUSB_LIBS debug ${LIBUSB_LIB_DEBUG} optimized ${LIBUSB_LIB_RELEASE})  
  
endif()

# ========================================================================================= SOURCES

set(cameraviewersource
  src/cameraviewer.cpp
  src/CameraPS3Eye.cpp
  src/ps3eye.cpp
  src/ps3eye_capi.cpp
  src/stdafx.cpp)

set(monosource
  src/singlecamcalibration.cpp
  src/CameraPS3Eye.cpp
  src/ps3eye.cpp
  src/ps3eye_capi.cpp
  src/stdafx.cpp)
  
set(monosourcefisheye
  src/singlecamcalibration_fisheye.cpp
  src/CameraPS3Eye.cpp
  src/ps3eye.cpp
  src/ps3eye_capi.cpp
  src/stdafx.cpp)
  
set(monosourceomnidir
  src/singlecamcalibration_omnidir.cpp
  src/CameraPS3Eye.cpp
  src/ps3eye.cpp
  src/ps3eye_capi.cpp
  src/stdafx.cpp)  

set(stereosource
  src/stereocamcalibration.cpp
  src/CameraPS3Eye.cpp
  src/ps3eye.cpp
  src/ps3eye_capi.cpp
  src/stdafx.cpp)
  
# ========================================================================================= HEADERS

set(cameraviewerheader
  include/Camera.h
  include/CameraPS3Eye.h
  include/libusb.h
  include/ps3eye.h
  include/ps3eye_capi.h
  include/stdafx.h)

set(monoheader
  include/Camera.h
  include/CameraPS3Eye.h
  include/libusb.h
  include/ps3eye.h
  include/ps3eye_capi.h
  include/stdafx.h)

set(stereoheader
  include/Camera.h
  include/CameraPS3Eye.h
  include/libusb.h
  include/ps3eye.h
  include/ps3eye_capi.h
  include/stdafx.h)

# ========================================================================================= PROJECT

include_directories(
  include/
  ${OpenCV_INCLUDES})
  
link_directories(
  ${lib_dir})

if(UNIX)
  set(ADDITIONAL_UNIX_LIBS X11)
  add_definitions("-std=c++11")
  add_definitions("-fPIC")
else()
  add_definitions(-D_CRT_SECURE_NO_WARNINGS)
  add_definitions(-D_WINSOCK_DEPRECATED_NO_WARNINGS)
  add_definitions(-D_SCL_SECURE_NO_WARNINGS)
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MDd")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MD")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")
  set(CMAKE_EXE_LINKER_FLAGS "/NODEFAULTLIB:LIBCMT /LTCG")
endif()

add_executable(cameraviewer ${cameraviewersource} ${cameraviewerheader})

add_executable(singlecamcalibration ${monosource} ${monoheader})

add_executable(stereocamcalibration ${stereosource} ${stereoheader}) 

target_link_libraries(cameraviewer 
  ${OpenCV_LIBS}
  ${LIBUSB_LIBS})

target_link_libraries(singlecamcalibration 
  ${OpenCV_LIBS}
  ${LIBUSB_LIBS})

target_link_libraries(stereocamcalibration 
  ${OpenCV_LIBS}
  ${LIBUSB_LIBS})

  if(WIN32)
  set_target_properties(cameraviewer PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_SOURCE_DIR}/bin
    RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_SOURCE_DIR}/bin)

  # Append "-d" to Debug Executable
  set_target_properties(cameraviewer PROPERTIES DEBUG_POSTFIX "-d")
endif()
  
if(WIN32)
  set_target_properties(singlecamcalibration PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_SOURCE_DIR}/bin
    RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_SOURCE_DIR}/bin)

  # Append "-d" to Debug Executable
  set_target_properties(singlecamcalibration PROPERTIES DEBUG_POSTFIX "-d")
endif()


if(WIN32)
  set_target_properties(stereocamcalibration PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_SOURCE_DIR}/bin
    RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_SOURCE_DIR}/bin)

  # Append "-d" to Debug Executable
  set_target_properties(stereocamcalibration PROPERTIES DEBUG_POSTFIX "-d")
endif()

# ===================================================================== Visual Studio Configuration

# Filters
if(WIN32)

# User Configuration (Working Directory)
  file(WRITE "${CMAKE_BINARY_DIR}/${PROJECT_NAME}.vcxproj.user" "\
<?xml version=\"1.0\" encoding=\"utf-8\"?>\n\
<Project ToolsVersion=\"12.0\" xmlns=\"http://schemas.microsoft.com/developer/msbuild/2003\">\n\
  <PropertyGroup Condition=\"'$(Configuration)|$(Platform)'=='Debug|x64'\">\n\
	<LocalDebuggerWorkingDirectory>$(ProjectDir)..\\</LocalDebuggerWorkingDirectory>\n\
	<DebuggerFlavor>WindowsLocalDebugger</DebuggerFlavor>\n\
  </PropertyGroup>\n\
  <PropertyGroup Condition=\"'$(Configuration)|$(Platform)'=='Release|x64'\">\n\
	<LocalDebuggerWorkingDirectory>$(ProjectDir)..\\</LocalDebuggerWorkingDirectory>\n\
	<DebuggerFlavor>WindowsLocalDebugger</DebuggerFlavor>\n\
  </PropertyGroup>\n\
</Project>")
endif()
