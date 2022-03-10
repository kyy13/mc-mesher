# mc-mesher
# Kyle J Burgess

add_executable(
	gm_boundary_conditions
	src/main.cpp
)

target_include_directories(
	gm_boundary_conditions
	PUBLIC
	${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(
	gm_boundary_conditions
	${PROJECT_NAME}
	-Wl,-allow-multiple-definition
)

add_test(
	NAME
	gm_boundary_conditions
	COMMAND
	gm_boundary_conditions
)

set_target_properties(
    gm_boundary_conditions
    PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS ON
)
