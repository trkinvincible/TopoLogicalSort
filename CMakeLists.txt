cmake_minimum_required(VERSION 3.10)
project(test_c_plusplus)

set(CMAKE_CXX_STANDARD 17)
#set(CMAKE_CXX_FLAGS "-pg") - use only with GCC 's own profiler

#list (APPEND CMAKE_P.REFIX_PATH "/media/trk/DATA/Code-Factory/NUTO/externals_libs")

SET( _SOURCES_
    ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
)

#flash logger
set(TESTS OFF CACHE INTERNAL "")
set(MICROSERVICE OFF CACHE INTERNAL "")

#boost C++
find_package(Boost COMPONENTS program_options REQUIRED)

include(FetchContent)
find_library(FLASHLOGGER_LIB
  NAMES FlashLogger
)
if(NOT FLASHLOGGER_LIB)
    set(FETCHCONTENT_QUIET OFF)
    FetchContent_Declare(
      FlashLogger
      GIT_REPOSITORY https://github.com/trkinvincible/FlashLoggerForCpp.git
      GIT_TAG        master
    )
    FetchContent_MakeAvailable(FlashLogger)
endif()


add_executable(${PROJECT_NAME} ${_SOURCES_})

#set (CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address")
#set (CMAKE_LINKER_FLAGS_DEBUG "${CMAKE_LINKER_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address")

if(MICROSERVICE)
    set(LINK_LIBRARIES ${Boost_LIBRARIES} grpc++)
else()
    set(LINK_LIBRARIES protobuf ${Boost_LIBRARIES})
endif(MICROSERVICE)
if(MICROSERVICE)
    target_link_libraries(${PROJECT_NAME}
        -lpthread
        -ltcmalloc
        -latomic
        -Wl,--no-as-needed -lprofiler
        -fno-builtin-malloc -fno-builtin-calloc -fno-builtin-realloc -fno-builtin-free
        ${LINK_LIBRARIES}
        gtest
        fl_grpc_proto
        ${_REFLECTION}
        ${_GRPC_GRPCPP}
        ${_PROTOBUF_LIBPROTOBUF})
else()
    target_link_libraries(${PROJECT_NAME}
        FlashLogger
        -lpthread
        -ltcmalloc
        -latomic
        -Wl,--no-as-needed -lprofiler
        -fno-builtin-malloc -fno-builtin-calloc -fno-builtin-realloc -fno-builtin-free
        ${LINK_LIBRARIES})
endif(MICROSERVICE)
