project(kanavi_node)

file(GLOB SOURCES *.cpp)

add_library( ${PROJECT_NAME} OBJECT
	${SOURCES}
)
