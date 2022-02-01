# MCmesher
# Kyle J Burgess

cmake_minimum_required(VERSION 3.7)
project(MCmesher)

option(BUILD_TESTS "set to on to build tests" ON)

# library
add_library(
    ${PROJECT_NAME}
    SHARED
    src/MCmesher.cpp
    src/GenerateMeshFN.cpp
    src/GenerateMeshVN.cpp
    src/LookupTable.cpp
)

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
target_link_libraries(
    ${PROJECT_NAME}
    -static-libgcc
    -static-libstdc++
)

IF(BUILD_TESTS)
    project(MCmesherTest)

    add_executable(
        ${PROJECT_NAME}
        test/src/main.cpp
    )

    target_include_directories(
        ${PROJECT_NAME}
        PUBLIC
        test/include
        "test/ext/gradient-noise/include"
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

    target_link_libraries(
        ${PROJECT_NAME}
        MCmesher
        -Wl,-allow-multiple-definition
    )

    set_target_properties(
        ${PROJECT_NAME}
        PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED ON
        CXX_EXTENSIONS ON
    )
ENDIF()
