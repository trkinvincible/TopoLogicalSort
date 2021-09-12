cmake_minimum_required(VERSION 3.10)
project(test_c_plusplus)

set(CMAKE_CXX_STANDARD 17)
#set(CMAKE_CXX_FLAGS "-pg") - use only with GCC 's own profiler

#list (APPEND CMAKE_P.REFIX_PATH "/media/trk/DATA/Code-Factory/NUTO/externals_libs")

SET( _SOURCES_
    ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
)

add_executable(${PROJECT_NAME} ${_SOURCES_})

#set (CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address")
#set (CMAKE_LINKER_FLAGS_DEBUG "${CMAKE_LINKER_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address")

target_link_libraries(${PROJECT_NAME}
        -latomic -pthread
        -lpthread -Wl,--no-as-needed
        -lprofiler -ltcmalloc
        -fno-builtin-malloc -fno-builtin-calloc -fno-builtin-realloc -fno-builtin-free
)
