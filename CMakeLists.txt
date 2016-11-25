cmake_minimum_required (VERSION 2.6)
project (NavXTimeStamp)

FIND_PACKAGE (Threads REQUIRED)



file(GLOB SOURCES
	"*.h"
	"*.cpp"
)

include_directories(".")

add_executable(NavXTimeStamp ${SOURCES})

target_link_libraries(NavXTimeStamp ${CMAKE_THREAD_LIBS_INIT})
target_compile_features(NavXTimeStamp PRIVATE cxx_range_for)

