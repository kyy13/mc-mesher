# MCmesher
# Kyle J Burgess

add_executable(
    gm_vn_unique_vertices
    src/main.cpp
)

target_include_directories(
    gm_vn_unique_vertices
    PUBLIC
    ${PROJECT_SOURCE_DIR}/include
    "../ext/gradient-noise/include"
)

IF (CMAKE_BUILD_TYPE MATCHES Debug)
    target_compile_options(
        gm_vn_unique_vertices
        PRIVATE
        -Wall
        -g
    )
ELSE()
    target_compile_options(
        gm_vn_unique_vertices
        PRIVATE
        -O3
    )
ENDIF()

target_link_libraries(
    gm_vn_unique_vertices
    ${PROJECT_NAME}
    -Wl,-allow-multiple-definition
)

add_test(
        NAME
        gm_vn_unique_vertices
        COMMAND
        gm_vn_unique_vertices
)

set_target_properties(
    gm_vn_unique_vertices
    PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS ON
)
