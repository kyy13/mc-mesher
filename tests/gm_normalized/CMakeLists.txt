# mc-mesher
# Kyle J Burgess

add_executable(
        gm_normalized
        src/main.cpp
)

target_include_directories(
        gm_normalized
        PUBLIC
        ${PROJECT_SOURCE_DIR}/include
        "../ext/gradient-noise/include"
)

target_link_libraries(
        gm_normalized
        ${PROJECT_NAME}
        -Wl,-allow-multiple-definition
)

add_test(
        NAME
        gm_normalized
        COMMAND
        gm_normalized
)

set_target_properties(
        gm_normalized
        PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED ON
        CXX_EXTENSIONS ON
)
