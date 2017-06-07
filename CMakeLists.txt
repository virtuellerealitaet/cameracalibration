cmake_minimum_required(VERSION 3.5)
project(cameracalibration)


# =========================================================================================
# OPTIONS
# =========================================================================================

set(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "" FORCE)

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)


# =========================================================================================
# PACKAGES
# =========================================================================================

# OpenGL
find_package(OpenGL REQUIRED)

# local project libraries
set(lib_dir ${CMAKE_SOURCE_DIR}/lib)

# OS-specific library configuration
if (UNIX)

  # OPENCV
  find_package(OpenCV 3.2 COMPONENTS highgui core imgcodecs imgproc features2d calib3d videoio REQUIRED)
  #set(OpenCV_INCLUDES /usr/local/include)
  #set(OpenCV_LIBS opencv_core opencv_highgui)

  # Boost
  #find_package(Boost)
  find_package(Boost 1.58 COMPONENTS program_options system filesystem regex REQUIRED)


  # Qt
  find_package(Qt5Widgets)
  find_package(Qt5Core)
  #find_package(Qt5a 5.9 COMPONENTS widgets core REQUIRED)



else()

  # OPENCV
  if (OpenCV_BASE_PATH)
    set(OpenCV_LIB_DEBUG ${OpenCV_BASE_PATH}/lib/opencv_world320d.lib CACHE FILEPATH "")
    set(OpenCV_LIB_RELEASE ${OpenCV_BASE_PATH}/lib/opencv_world320.lib CACHE FILEPATH "")
    set(OpenCV_LIBS debug ${OpenCV_LIB_DEBUG} optimized ${OpenCV_LIB_RELEASE})
    set(OpenCV_INCLUDES ${OpenCV_BASE_PATH}/include/ CACHE PATH "")
  endif()
  set(OpenCV_BASE_PATH $ENV{OPENCV_310} CACHE PATH "")

  # LIBUSB
  #find_package(libusb REQUIRED)
  
  set(LIBUSB_LIB_DEBUG ../lib/libusbd CACHE FILEPATH "")
  set(LIBUSB_LIB_RELEASE ../lib/libusb CACHE FILEPATH "")
  set(LIBUSB_LIBS debug ${LIBUSB_LIB_DEBUG} optimized ${LIBUSB_LIB_RELEASE})
  
  # Qt - not required

  # BOOST - not required

endif()

# ========================================================================================= SOURCES

set(simplecamsource
  src/simplecam.cpp
  src/stdafx.cpp)

set(cameraviewersource
  src/cameraviewer.cpp
  src/ps3eye.cpp
  src/ps3eye_capi.cpp
  src/stdafx.cpp)

set(monosource
  src/singlecamcalibration.cpp
  src/ps3eye.cpp
  src/ps3eye_capi.cpp
  src/stdafx.cpp)
  
set(monosourcefisheye
  src/singlecamcalibration_fisheye.cpp
  src/ps3eye.cpp
  src/ps3eye_capi.cpp
  src/stdafx.cpp)
  
set(monosourceomnidir
  src/singlecamcalibration_omnidir.cpp
  src/ps3eye.cpp
  src/ps3eye_capi.cpp
  src/stdafx.cpp)  

set(stereosource
  src/stereocamcalibration.cpp
  src/ps3eye.cpp
  src/ps3eye_capi.cpp
  src/stdafx.cpp)
  
# ========================================================================================= HEADERS

set(simplecamheader
  include/Camera.h
  include/ThreadCamera.h
  include/stdafx.h)

set(cameraviewerheader
  include/Camera.h
  include/ThreadCamera.h
  include/libusb.h
  include/ps3eye.h
  include/ps3eye_capi.h
  include/stdafx.h)

set(monoheader
  include/Camera.h
  include/ThreadCamera.h
  include/libusb.h
  include/ps3eye.h
  include/ps3eye_capi.h
  include/stdafx.h)

set(stereoheader
  include/Camera.h
  include/ThreadCamera.h
  include/libusb.h
  include/ps3eye.h
  include/ps3eye_capi.h
  include/stdafx.h)

# ========================================================================================= PROJECT


MESSAGE( STATUS "Boost_INCLUDE_DIR: " ${Boost_INCLUDE_DIR} )
MESSAGE( STATUS "Boost_LIBRARY_DIR: " ${Boost_LIBRARY_DIR} )
MESSAGE( STATUS "Boost_LIBRARIES: " ${Boost_LIBRARIES} )

MESSAGE( STATUS "OpenCV_INCLUDE_DIR: " ${OpenCV_INCLUDE_DIR} )
MESSAGE( STATUS "OpenCV_INCLUDES: " ${OpenCV_INCLUDES} )
MESSAGE( STATUS "OpenCV_LIBRARY_DIR: " ${OpenCV_LIBRARY_DIR} )
MESSAGE( STATUS "OpenCV_LIBRARIES: " ${OpenCV_LIBRARIES} )

MESSAGE( STATUS "QT5_INCLUDES: " ${QT5_INCLUDES} )
MESSAGE( STATUS "QT5_INCLUDE_DIR: " ${QT5_INCLUDE_DIR} )
MESSAGE( STATUS "QT5_LIBRARIES: " ${QT5_LIBRARIES} )


include_directories(
  include/
  ${OpenCV_INCLUDES}
  ${Boost_INCLUDE_DIR})
  
MESSAGE( STATUS "all includes: " ${include_directories} )

link_directories(
  ${lib_dir}
  ${Boost_LIBRARY_DIR})

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


# =====================================================================
# Unix programs configuration
# =====================================================================

if(UNIX)

add_executable(simplecam ${simplecamsource} ${simplecamheader})

#add_executable(cameraviewer ${cameraviewersource} ${cameraviewerheader})

#add_executable(singlecamcalibration ${monosource} ${monoheader})

#add_executable(stereocamcalibration ${stereosource} ${stereoheader})

target_link_libraries(simplecam
  ${OpenCV_LIBS}
  Qt5::Widgets
  Qt5::Core
  ${Boost_LIBRARIES}
  pthread
  #SDL2
  #readline
  )


#APR_ONLY_LIBS = -lapr-1 -lpthread


add_definitions(-DUNIX)

endif()

# =====================================================================
# Win32 programs configuration
# =====================================================================

if(WIN32)

#
# SIMPLE CAMERA VIEWER
#
#
add_executable(simplecam src/simplecam.cpp)
#
target_link_libraries(simplecam ${OpenCV_LIBS} Qt5::Widgets Qt5::Core)
# set output path
set_target_properties(simplecam PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_SOURCE_DIR}/bin
    RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_SOURCE_DIR}/bin)
# Append "-d" to Debug Executable
set_target_properties(simplecam PROPERTIES DEBUG_POSTFIX "-d")


#
# CAMERA VIEWER
#
add_executable(cameraviewer ${cameraviewersource} ${cameraviewerheader})
# add libraries
target_link_libraries(cameraviewer ${OpenCV_LIBS} ${LIBUSB_LIBS})
# set output path
set_target_properties(cameraviewer PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_SOURCE_DIR}/bin
    RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_SOURCE_DIR}/bin)
# Append "-d" to Debug Executable
set_target_properties(cameraviewer PROPERTIES DEBUG_POSTFIX "-d")


#
# SINGLE CAMERA CALIBRATION
#
add_executable(singlecamcalibration ${monosource} ${monoheader})
#
target_link_libraries(singlecamcalibration ${OpenCV_LIBS} ${LIBUSB_LIBS})
#
set_target_properties(singlecamcalibration PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_SOURCE_DIR}/bin
    RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_SOURCE_DIR}/bin)
# Append "-d" to Debug Executable
set_target_properties(singlecamcalibration PROPERTIES DEBUG_POSTFIX "-d")


#
# STEREO CAMERA CALIBRATION
#
add_executable(stereocamcalibration ${stereosource} ${stereoheader})
#
target_link_libraries(stereocamcalibration ${OpenCV_LIBS} ${LIBUSB_LIBS})
#
set_target_properties(singlecamcalibration PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_SOURCE_DIR}/bin
    RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_SOURCE_DIR}/bin)
# Append "-d" to Debug Executable
set_target_properties(singlecamcalibration PROPERTIES DEBUG_POSTFIX "-d")

 endif()


# =====================================================================
# Visual Studio Configuration
# =====================================================================

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
