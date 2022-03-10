# mc-mesher
# Kyle J Burgess

add_executable(
        gm_edge_cases
        src/main.cpp
)

target_include_directories(
        gm_edge_cases
        PUBLIC
        ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(
        gm_edge_cases
        ${PROJECT_NAME}
        -Wl,-allow-multiple-definition
)

add_test(
        NAME
        gm_edge_cases
        COMMAND
        gm_edge_cases
)

set_target_properties(
        gm_edge_cases
        PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED ON
        CXX_EXTENSIONS ON
)
