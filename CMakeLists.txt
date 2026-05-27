cmake_minimum_required(VERSION 3.19)
project(omp-pathfinder LANGUAGES C CXX VERSION 1.1.0)
set(CMAKE_CXX_STANDARD 17)

set(CMAKE_POLICY_VERSION_MINIMUM 3.5)

if(UNIX AND NOT APPLE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -m32 -msse2 -mfpmath=sse")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -m32 -msse2 -mfpmath=sse")
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -m32")
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Release" OR CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -flto -funroll-loops")
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -flto -funroll-loops")
    endif()
endif()

add_subdirectory(deps/omp-sdk)

include_directories(
    .
    deps
    deps/omp-sdk/include
    deps/pawn/source
    deps/pawn/source/linux
    deps/pawn-natives
)

add_definitions(
    -DHAVE_STDINT_H=1
    -DPAWN_CELL_SIZE=32
)

add_library(${PROJECT_NAME} SHARED
    main.cc
    natives.cc
    pathfinder-component.cc
)

target_link_libraries(${PROJECT_NAME} PRIVATE OMP-SDK)
set_target_properties(${PROJECT_NAME} PROPERTIES PREFIX "")