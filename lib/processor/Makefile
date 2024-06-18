#UDP library makefile
#source code Path
SRC_HOME=./

#Library Name
LIB_BASIC:=libKanaviProcessor
LIB_NAME:=
LIB_PATH=../bin
#define Include Path
INC_HOME=.

LIBS=-L${LIB_PATH} -lKanaviUDP -lKanaviParser -lKanavi_VL_AS16

#compile flags
#CFLAGS: C compile flag
CFLAGS=-g -std=c++11 -I${INC_HOME} -c -Wno-deprecated -fPIC

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
# OBJS=kanaviProcessor.o

#command --all
all: ${LIB_NAME}	#dependency *.so file -> 1.

#generate *.so file
${LIB_NAME}: ${OBJS}	#dependency *.o file -> 2.
	g++ ${CXXFLAGS} -o $@ ${OBJS} ${LIBS}	#compile -> 4.
	chmod +x $@					#Authorization -> 5.
	mv $@ ../bin				#move *.so file -> 6.

.cpp.o:					#compile *.o using *.cpp ->3.
	g++ ${CFLAGS} $<

clean:					#clean 
	@rm -rf *.o
