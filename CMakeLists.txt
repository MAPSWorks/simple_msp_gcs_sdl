cmake_minimum_required( VERSION 2.8.12 )
project( simple_msp_gcs )

include( nanogui_sdl_include.cmake )

SET( SRCS )
aux_source_directory( "./serial_setup" SRCS )
aux_source_directory( "./msp_protocol" SRCS)
aux_source_directory( "./gui" SRCS )
aux_source_directory( "./rpi-udp-stream-client/computer_vision" SRCS )
aux_source_directory( "./rpi-udp-stream-client/udp_setup" SRCS )
aux_source_directory( "./rpi-udp-stream-client/common_util" SRCS )
aux_source_directory( "./rpi-udp-stream-client/ffmpeg_setup" SRCS )
aux_source_directory( "./app" SRCS )
add_executable( ${CMAKE_PROJECT_NAME} ${SRCS} )

find_package( OpenCV REQUIRED )

set( GCC_COVERAGE_COMPILE_FLAGS -DSTANDALONE -D__STDC_CONSTANT_MACROS -pipe -g -Wall -Wno-unused-result -Wno-deprecated-declarations )
set( GCC_COVERAGE_LINK_FLAGS -lpthread -lSDL2main -lSDL2 -lavformat -lavcodec -lavutil -lswscale )

target_compile_options( ${CMAKE_PROJECT_NAME} PRIVATE ${GCC_COVERAGE_COMPILE_FLAGS} )
target_link_libraries( ${CMAKE_PROJECT_NAME} ${GCC_COVERAGE_LINK_FLAGS} nanogui ${NANOGUISDL_EXTRA_LIBS} ${OpenCV_LIBS} )
