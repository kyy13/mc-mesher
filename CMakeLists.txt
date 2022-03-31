# mc-mesher
# Kyle J Burgess

cmake_minimum_required(VERSION 3.7)
project(mcmesher)

option(MCM_BUILD_STATIC "build static?" OFF)
option(MCM_BUILD_TESTS "build tests?" ON)
option(MCM_STATIC_GCC "static link gcc?" ON)

# sources
set(
    MCM_SOURCES
    src/mc_mesher.cpp
    src/McmRayIntersectVirtualMesh.cpp
    src/McmLookupTable.cpp
    src/McmCommon.cpp
)

# library
IF (MCM_BUILD_STATIC)
    add_library(
        ${PROJECT_NAME}
        STATIC
        ${MCM_SOURCES}
    )
ELSE()
    add_library(
        ${PROJECT_NAME}
        SHARED
        ${MCM_SOURCES}
    )
ENDIF()

# includes
target_include_directories(
    ${PROJECT_NAME}
    PUBLIC
    include
)

IF (CMAKE_BUILD_TYPE MATCHES Debug)
    target_compile_options(
        ${PROJECT_NAME}
        PRIVATE
        -Wall
        -g
    )
ELSE()
    target_compile_options(
        ${PROJECT_NAME}
        PRIVATE
        -O3
    )
ENDIF()

# c++ version
set_target_properties(
    ${PROJECT_NAME}
    PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS ON
)

# link libraries
IF(MCM_STATIC_GCC)
    target_link_libraries(
        ${PROJECT_NAME}
        -static-libgcc
        -static-libstdc++
        -Wl,-Bstatic,--whole-archive -lwinpthread -Wl,--no-whole-archive
    )
ENDIF()

IF(MCM_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
ENDIF()
