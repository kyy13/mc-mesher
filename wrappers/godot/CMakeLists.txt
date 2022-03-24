# mc-mesher
# Kyle J Burgess

cmake_minimum_required(VERSION 3.7)
project(mcmesher-gd)

set(MCM_BUILD_STATIC ON)
set(MCM_BUILD_TESTS OFF)

# library
add_library(
	${PROJECT_NAME}
	SHARED
	src/mc_mesher.cpp
)

# includes
target_include_directories(
	${PROJECT_NAME}
    PUBLIC
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/ext/godot-cpp/include/godot_cpp/core
    ${PROJECT_SOURCE_DIR}/ext/godot-cpp/godot-headers
    ${PROJECT_SOURCE_DIR}/ext/godot-cpp/include/godot_cpp
)

add_subdirectory(
	ext/godot-cpp
)

IF (CMAKE_BUILD_TYPE MATCHES Debug)
    target_compile_options(
        ${PROJECT_NAME}
        PRIVATE
        -Wall
        -g
    )

	add_subdirectory(
		${PROJECT_SOURCE_DIR}/../../
		${PROJECT_BINARY_DIR}/mcmesher/build/Debug
	)
ELSE()
    target_compile_options(
        ${PROJECT_NAME}
        PRIVATE
        -O3
    )

    add_subdirectory(
		${PROJECT_SOURCE_DIR}/../../
		${PROJECT_BINARY_DIR}/mcmesher/build/Release
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
    PRIVATE
    mcmesher
    godot-cpp
)
