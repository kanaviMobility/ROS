OS_NAME=$(OS)
UNAME_S:= $(shell uname -s)

ifeq ( $(OS), WIndows_NT )
	OS_NAME="windows"
else
ifeq (${UNAME_S},Linux)
	OS_NAME="linux"
	FORM=*.so
endif
ifeq ($(UNAME_S),Darwin)
	OS_NAME="mac"
	FORM=*.dylib
endif
endif

clean:
	@rm -rf ${FORM}

all:
	echo  "nothing"
#	sudo mkdir /usr/lib/KANAVI
#	sudo cp *.so /usr/lib/KANAVI
