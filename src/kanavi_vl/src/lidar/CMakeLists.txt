project(kanavi_lidar)

file(GLOB SOURCES *.cpp)

add_library( ${PROJECT_NAME} OBJECT
	${SOURCES}
)
