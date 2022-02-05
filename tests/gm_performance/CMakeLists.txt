# MCmesher
# Kyle J Burgess

add_executable(
    gm_performance_test
    src/main.cpp
)

target_include_directories(
    gm_performance_test
    PUBLIC
    ${PROJECT_SOURCE_DIR}/include
    "../ext/gradient-noise/include"
)

IF (CMAKE_BUILD_TYPE MATCHES Debug)
    target_compile_options(
        gm_performance_test
        PRIVATE
        -Wall
        -g
    )
ELSE()
    target_compile_options(
        gm_performance_test
        PRIVATE
        -O3
    )
ENDIF()

target_link_libraries(
    gm_performance_test
    ${PROJECT_NAME}
    -Wl,-allow-multiple-definition
)

set_target_properties(
    gm_performance_test
    PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS ON
)
