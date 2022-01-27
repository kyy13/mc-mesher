# MCmesher
# Kyle J Burgess

cmake_minimum_required(VERSION 3.7)
project(MCmesher)

# library
add_library(
    ${PROJECT_NAME}
    SHARED
    src/MCmesher.cpp
)

add_executable(
    MCmesherTest
    src/main.cpp
)

# includes
target_include_directories(
    ${PROJECT_NAME}
    PUBLIC
    include
)

# compiler options
IF (CMAKE_BUILD_TYPE MATCHES Debug)
    add_compile_options(
        MCmesherTest
        PUBLIC
        -Wall
        -g
    )
ELSE()
    add_compile_options(
        MCmesherTest
        PUBLIC
        -O3
    )
ENDIF()

# c++ version
set_target_properties(
    ${PROJECT_NAME}
    MCmesherTest
    PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS ON
)

# link libraries
target_link_libraries(
    ${PROJECT_NAME}
    -static-libgcc
    -static-libstdc++
)

target_link_libraries(
    MCmesherTest
    ${PROJECT_NAME}
    -Wl,-allow-multiple-definition
)
