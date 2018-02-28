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

  #OpenCV
  find_package(OpenCV 3.2 COMPONENTS highgui core imgcodecs imgproc features2d calib3d videoio REQUIRED)
  if (OpenCV_FOUND)
    MESSAGE(${OpenCV_LIBS})
  endif()

  #Boost
  #find_package(Boost 1.58 COMPONENTS program_options system filesystem regex REQUIRED)
  #if (Boost_FOUND)
  #    MESSAGE("BOOST FOUND")
  #    MESSAGE(${Boost_INCLUDE_DIR})
  #    MESSAGE(${Boost_LIBRARIES})
  #endif()


  # Qt
  #find_package(Qt5Widgets)
  #find_package(Qt5Core)

  
  #Boost
  #find_package(Boost 1.58 COMPONENTS program_options system filesystem regex REQUIRED)
  #if (Boost_FOUND)
  #    MESSAGE("BOOST FOUND")
  #    MESSAGE(${Boost_INCLUDE_DIR})
  #    MESSAGE(${Boost_LIBRARIES})
  #endif()

  #OpenCV
  find_package(OpenCV 3.2 COMPONENTS highgui core imgcodecs imgproc features2d calib3d videoio REQUIRED)
  if (OpenCV_FOUND)
    MESSAGE(${OpenCV_LIBS})
  endif()

  set(OpenCV_LIBS_DEBUG ${OpenCV_LIBS})  


else()

  # OPENCV
  if (OpenCV_BASE_PATH)
    set(OpenCV_LIB_DEBUG ${OpenCV_BASE_PATH}/x64/vc14/lib/opencv_world300d.lib CACHE FILEPATH "")
    set(OpenCV_LIB_RELEASE ${OpenCV_BASE_PATH}/x64/vc14/lib/opencv_world300.lib CACHE FILEPATH "")
    set(OpenCV_LIBS debug ${OpenCV_LIB_DEBUG} optimized ${OpenCV_LIB_RELEASE})
    set(OpenCV_INCLUDES ${OpenCV_BASE_PATH}/include/ CACHE PATH "")
  endif()
  #set(OpenCV_BASE_PATH $ENV{OPENCV_320} CACHE PATH "")
  set(OpenCV_BASE_PATH "/opencv3/opencv/build" CACHE PATH "")

  

  # LIBUSB
  set(USB_LIB_DEBUG ${lib_dir}/libusbd.lib)
  set(USB_LIB_RELEASE ${lib_dir}/libusb.lib)
  set(USB_LIBS debug ${USB_LIB_DEBUG} optimized ${USB_LIB_RELEASE})
  
  # Qt - not required
  
  # BOOST - not required

endif()

# ========================================================================================= SOURCES

set(ps3_source_windows
  src/ps3eye.cpp
  src/ps3eye_capi.cpp)

set(multicamviewersource
  src/multicamviewer.cpp
  src/stdafx.cpp)

set(singlecamviewersource
  src/viewer_singlecam.cpp
  src/stdafx.cpp)
  
set(multicamwritersource
  src/multicamwriter.cpp
  src/stdafx.cpp)

set(singlecamcalibsource
  src/singlecamcalibration.cpp
  src/stdafx.cpp)

set(stereocamcalibsource
  src/stereocamcalibration.cpp
  src/stdafx.cpp)
  
# ========================================================================================= HEADERS

set(ps3_header_windows
  include/ps3eye.h
  include/ps3eye_capi.h
  include/libusb.h)

set(singlecamviewerheader
  include/Camera.h
  include/ThreadCamera.h
  include/stdafx.h)  
  
set(multicamviewerheader
  include/Camera.h
  include/ThreadCamera.h
  include/stdafx.h)

set(multicamwriterheader
  include/Camera.h
  include/ThreadCamera.h
  include/stdafx.h)

set(singlecamcalibheader
  include/Camera.h
  include/ThreadCamera.h
  include/stdafx.h)

set(stereocamcalibheader
  include/Camera.h
  include/ThreadCamera.h
  include/stdafx.h)

# ========================================================================================= PROJECT

MESSAGE( STATUS "OpenCV_INCLUDE_DIR: " ${OpenCV_INCLUDE_DIR} )
MESSAGE( STATUS "OpenCV_INCLUDES: " ${OpenCV_INCLUDES} )
MESSAGE( STATUS "OpenCV_LIBRARY_DIR: " ${OpenCV_LIBRARY_DIR} )
MESSAGE( STATUS "OpenCV_LIBRARIES: " ${OpenCV_LIBRARIES} )

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

	add_executable(singlecamviewer ${singlecamviewersource} ${singlecamviewerheader})
	target_link_libraries(singlecamviewer
	  ${OpenCV_LIBS}
	  Qt5::Widgets
	  Qt5::Core
	  ${Boost_LIBRARIES}
	  pthread
	  )

	add_executable(multicamviewer ${multicamviewersource} ${multicamviewerheader})
        #set_target_properties(multicamviewer PROPERTIES LINKER_LANGUAGE CXX)
	target_link_libraries(multicamviewer
	  ${OpenCV_LIBS}
	  Qt5::Widgets
	  Qt5::Core
	  ${Boost_LIBRARIES}
	  pthread
	  )
	  
        add_executable(multicamwriter ${multicamwritersource} ${multicamwriterheader})

        target_link_libraries(multicamwriter
          ${OpenCV_LIBS}
          Qt5::Widgets
          Qt5::Core
          ${Boost_LIBRARIES}
          pthread
          )
	  
        add_executable(singlecamcalibration ${singlecamcalibsource} ${singlecamcalibheader})

        target_link_libraries(singlecamcalibration
          ${OpenCV_LIBS}
          Qt5::Widgets
          Qt5::Core
          ${Boost_LIBRARIES}
          pthread
          )

        add_executable(stereocamcalibration ${stereocamcalibsource} ${stereocamcalibheader})

        target_link_libraries(stereocamcalibration
          ${OpenCV_LIBS}
          Qt5::Widgets
          Qt5::Core
          ${Boost_LIBRARIES}
          pthread
          )

	add_definitions(-DUNIX)

endif()

# =====================================================================
# Win32 programs configuration
# =====================================================================

if(WIN32)

	link_directories(
		${lib_dir}
	)

	#
	# SINGLE CAMERA VIEWER
	#
	#
	add_executable(singlecamviewer ${singlecamviewersource} ${singlecamviewerheader} ${ps3_header_windows} ${ps3_source_windows})
	#
	target_link_libraries(singlecamviewer ${OpenCV_LIBS} ${USB_LIBS})
	# set output path
	set_target_properties(singlecamviewer PROPERTIES
		RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_SOURCE_DIR}/bin
		RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_SOURCE_DIR}/bin)
	# Append "-d" to Debug Executable
	set_target_properties(singlecamviewer PROPERTIES DEBUG_POSTFIX "-d")

	#
	# MULTI CAMERA VIEWER
	#
	#
	add_executable(multicamviewer ${multicamviewersource} ${multicamviewerheader} ${ps3_header_windows} ${ps3_source_windows})
	#
	target_link_libraries(multicamviewer ${OpenCV_LIBS} ${USB_LIBS})
	# set output path
	set_target_properties(multicamviewer PROPERTIES
		RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_SOURCE_DIR}/bin
		RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_SOURCE_DIR}/bin)
	# Append "-d" to Debug Executable
	set_target_properties(multicamviewer PROPERTIES DEBUG_POSTFIX "-d")


	#
	# MULTI CAMERA WRITER
	#
	add_executable(multicamwriter ${multicamwritersource} ${multicamwriterheader} ${ps3_header_windows} ${ps3_source_windows})
	# add libraries
	target_link_libraries(multicamwriter ${OpenCV_LIBS} ${USB_LIBS})
	# set output path
	set_target_properties(multicamwriter PROPERTIES
		RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_SOURCE_DIR}/bin
		RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_SOURCE_DIR}/bin)
	# Append "-d" to Debug Executable
	set_target_properties(multicamwriter PROPERTIES DEBUG_POSTFIX "-d")


	#
	# SINGLE CAMERA CALIBRATION
	#
	add_executable(singlecamcalibration ${singlecamcalibsource} ${singlecamcalibheader} ${ps3_header_windows} ${ps3_source_windows})
	#
	target_link_libraries(singlecamcalibration ${OpenCV_LIBS} ${USB_LIBS})
	#
	set_target_properties(singlecamcalibration PROPERTIES
		RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_SOURCE_DIR}/bin
		RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_SOURCE_DIR}/bin)
	# Append "-d" to Debug Executable
	set_target_properties(singlecamcalibration PROPERTIES DEBUG_POSTFIX "-d")


	#
	# STEREO CAMERA CALIBRATION
	#
	add_executable(stereocamcalibration ${stereocamcalibsource} ${stereocamcalibheader} ${ps3_header_windows} ${ps3_source_windows})
	#
	target_link_libraries(stereocamcalibration ${OpenCV_LIBS} ${USB_LIBS})
	#
	set_target_properties(stereocamcalibration PROPERTIES
		RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_SOURCE_DIR}/bin
		RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_SOURCE_DIR}/bin)
	# Append "-d" to Debug Executable
	set_target_properties(stereocamcalibration PROPERTIES DEBUG_POSTFIX "-d")

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
