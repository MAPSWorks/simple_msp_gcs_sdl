if(NANOGUI_GL_IMPLEMENTATION MATCHES "gl2" OR NOT DEFINED NANOGUI_GL_IMPLEMENTATION)
    add_definitions(-DNANOVG_GL2_IMPLEMENTATION)
elseif(NANOGUI_GL_IMPLEMENTATION MATCHES "gl3")
    add_definitions(-DNANOVG_GL3_IMPLEMENTATION)
elseif(NANOGUI_GL_IMPLEMENTATION MATCHES "gles2")
    add_definitions(-DNANOVG_GLES2_IMPLEMENTATION)
elseif(NANOGUI_GL_IMPLEMENTATION MATCHES "gles3")
    add_definitions(-DNANOVG_GLES3_IMPLEMENTATION)
else()
    message(FATAL_ERROR "Unknown GL implementaion. Use one of gl2, gl3, gles2, gles3.")
endif()

set(NANOGUISDL_EXTRA_LIBS "")

include(CheckCXXCompilerFlag)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'Release' as none was specified.")
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
    "MinSizeRel" "RelWithDebInfo")
endif()
string(TOUPPER "${CMAKE_BUILD_TYPE}" U_CMAKE_BUILD_TYPE)

if (CMAKE_CXX_COMPILER_ID MATCHES "Clang" OR CMAKE_CXX_COMPILER_ID MATCHES "GNU")
  # Enable C++11 mode on GCC / Clang
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
  add_definitions( -DEIGEN_DONT_ALIGN )
endif()

if(CMAKE_SYSTEM MATCHES "Linux")
  if(NANOGUI_GL_IMPLEMENTATION MATCHES "gles2" OR NANOGUI_GL_IMPLEMENTATION MATCHES "gles3")
    list(APPEND NANOGUISDL_EXTRA_LIBS GLESv2)
  else()
    list(APPEND NANOGUISDL_EXTRA_LIBS GL)
  endif()
  list(APPEND NANOGUISDL_EXTRA_LIBS Xxf86vm Xrandr Xinerama Xcursor Xi X11 pthread dl rt SDL2)
  add_definitions(-DNANOGUI_LINUX)
endif()

if (CMAKE_COMPILER_IS_GNUCC)
  set_source_files_properties( nanogui-sdl/ext/nanovg/src/nanovg.c PROPERTIES COMPILE_FLAGS -Wno-unused-result)
elseif(MSVC)
  set_source_files_properties( nanogui-sdl/ext/nanovg/src/nanovg.c PROPERTIES COMPILE_FLAGS "/wd4005 /wd4456 /wd4457")
endif()

link_directories( ${CMAKE_CURRENT_SOURCE_DIR}/nanogui-sdl )
include_directories( nanogui-sdl/include )
include_directories( nanogui-sdl/ext/eigen )
include_directories( nanogui-sdl/ext )

