# mc-mesher
# Kyle J Burgess

add_executable(
        gm_equal_mesh
        src/main.cpp
)

target_include_directories(
        gm_equal_mesh
        PUBLIC
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/src
        "../ext/gradient-noise/include"
)

target_link_libraries(
        gm_equal_mesh
        ${PROJECT_NAME}
        -Wl,-allow-multiple-definition
)

add_test(
        NAME
        gm_equal_mesh
        COMMAND
        gm_equal_mesh
)

set_target_properties(
        gm_equal_mesh
        PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED ON
        CXX_EXTENSIONS ON
)
