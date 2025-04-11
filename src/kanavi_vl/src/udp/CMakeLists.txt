project(kanavi_udp)

file(GLOB SOURCES *.cpp)

add_library( ${PROJECT_NAME} OBJECT 
			${SOURCES} )