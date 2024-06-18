
#source code Path
SRC_HOME=./
#define Include Path
INC_HOME=.

LIB_PATH=../../bin
LIB_BASIC:=libKanaviParser
LIB_NAME:=
CXXFLAGS:=
CFLAGS:=-g -std=c++11 -I${SRC_HOME} -c -Wno-deprecated -fPIC

#Library Name
LIBS=-L${LIB_PATH} -lKanavi_VL_AS16 -lKanavi_Industrial

OS_NAME=$(OS)
UNAME_S:= $(shell uname -s)

ifeq ( $(OS), WIndows_NT )
	OS_NAME="windows"
else
ifeq (${UNAME_S},Linux)
	OS_NAME="linux"
	FORM=.so
	LIB_NAME=${LIB_BASIC}${FORM}
	CXXFLAGS= -shared
endif
ifeq ($(UNAME_S),Darwin)
	OS_NAME="mac"
	FORM=.dylib
	LIB_NAME=${LIB_BASIC}${FORM}
	# CXXFLAGS= -dynamiclib -undefined suppress -flat_namespace
	CXXFLAGS= -shared
endif
endif
#find .cpp file
vpath %.cpp ${SRC_HOME}
#find .h file
vpath %.h	${SRC_HOME}

#static link(?) NAME
SRCS=$(wildcard *.cpp)
OBJS=$(SRCS:.cpp=.o)
# OBJS=lidarParser.o

#command --all
all: ${LIB_NAME}	#dependency *.so file -> 1.

#generate *.so file
${LIB_NAME}: ${OBJS}	#dependency *.o file -> 2.
# ifeq ($(UNAME_S),Darwin)
# 	export DYLD_LIBRARY_PATH=${LIB_PATH}
# endif
	g++ ${CXXFLAGS} -o $@ ${OBJS} ${LIBS}	#compile -> 4.
	chmod +x $@					#Authorization -> 5.
	mv $@ ../../bin				#move *.so file -> 6.

.cpp.o:					#compile *.o using *.cpp ->3.
	g++ ${CFLAGS} $<

clean:					#clean 
	@rm -rf *.o
