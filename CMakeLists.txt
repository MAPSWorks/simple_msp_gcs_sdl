cmake_minimum_required( VERSION 2.8.12 )
project( simple_msp_gcs )
SET( SRCS )
aux_source_directory( "./serial_setup" SRCS )
aux_source_directory( "./msp_protocol" SRCS)
aux_source_directory( "." SRCS)
add_executable( ${CMAKE_PROJECT_NAME} ${SRCS} )

set( GCC_COVERAGE_COMPILE_FLAGS -DSTANDALONE -D__STDC_CONSTANT_MACROS -pipe -Werror -g -Wall )
set( GCC_COVERAGE_LINK_FLAGS )

target_compile_options( ${CMAKE_PROJECT_NAME} PRIVATE ${GCC_COVERAGE_COMPILE_FLAGS} )
target_link_libraries( ${CMAKE_PROJECT_NAME} ${GCC_COVERAGE_LINK_FLAGS} )
