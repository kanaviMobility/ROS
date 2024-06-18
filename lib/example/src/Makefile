#main source code Path
SRC_HOME=.

#path&name for APP
APP_NAME=test_app

#path&name for Lobrary
LIB_PATH=/usr/lib

LIBS= -lpthread

LIB_INC_PATH=${INC_PATH}

KANAVI_PATH=$(shell pkg-config --cflags kanavi_lib)
KANAVI_LIB=$(shell pkg-config --libs kanavi_lib)
OPENCV_PATH=$(shell pkg-config --cflags opencv4)
OPENCV_LIB=$(shell pkg-config --libs opencv4)

#find source code file
vpath %.h	${SRC_HOME}
vpath %.cpp	${SRC_HOME}

vpath %.h	${KANAVI_PATH}/UDP
vpath %.h	${KANAVI_PATH}/LiDAR
vpath %.h	${KANAVI_PATH}/processor
vpath %.h	${KANAVI_PATH}/include

#set object Name
OBJS=main.o

#command : all
all: ${APP_NAME}		#dependency active file

#generate & compile active file file
${APP_NAME}: ${OBJS}	#dependency *.o
	g++ -o $@ $< ${LIBS} ${OPENCV_LIB} ${KANAVI_PATH} ${KANAVI_LIB}

#generate & compile *.o file
.cpp.o:					#generate *.o using .cpp
	g++ -std=c++11 -c -I${LIB_INC_PATH} ${OPENCV_PATH} ${KANAVI_PATH} ${KANAVI_LIB} $<

clean:
	@rm -rf *.o
	rm -rf ${APP_NAME}
