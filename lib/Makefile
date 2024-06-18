prefix=/usr/local
LIB_PATH=${prefix}/lib

INC_HOME=${prefix}/include
INC_PATH=${INC_HOME}/KANAVI_LIB

PKG_PATH=${LIB_PATH}/pkgconfig

LiDAR=${INC_PATH}/LiDAR
UDP=${INC_PATH}/UDP
include=${INC_PATH}/include
processor=${INC_PATH}/processor
VL_AS16=${LiDAR}/VL_AS16
INDUSTRIAL=${LiDAR}/industrial
parser=${LiDAR}/parser

#check Os
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

install: all
	@echo "mkdir lib"
	sudo mkdir -p ${LiDAR}
	sudo mkdir -p ${UDP}
	sudo mkdir -p ${include}
	sudo mkdir -p ${processor}
	sudo mkdir -p ${VL_AS16}
	sudo mkdir -p ${INDUSTRIAL}
	sudo mkdir -p ${parser}
	
	@echo "copy file"
	sudo cp UDP/*.h UDP/*.cpp UDP/*.o ${UDP}
	sudo cp processor/*.h processor/*.cpp processor/*.o ${processor}
	sudo cp include/*.h ${include}
	sudo cp LiDAR/VL_AS16/*.h LiDAR/VL_AS16/*.cpp LiDAR/VL_AS16/*.o ${VL_AS16}
	sudo cp LiDAR/industrial/*.h LiDAR/industrial/*.cpp LiDAR/industrial/*.o ${INDUSTRIAL}
	sudo cp LiDAR/parser/*.h LiDAR/parser/*.cpp LiDAR/parser/*.o ${parser}
	
	echo ${.}
ifeq ($(UNAME_S),Darwin)
	OK=$(shell if [ -d ${PKG_PATH} ]; then echo "ok"; fi)
ifneq ($(OK),ok)
	sudo mkdir -p ${PKG_PATH}
endif
endif
	sudo cp -f -p bin/${FORM} ${LIB_PATH}
	sudo cp -f -p pkgconfig/*.pc ${PKG_PATH}

all:
	@(cd bin;	make all)
	@(cd UDP;	make all)
	@(cd LiDAR;	make all)
	@(cd processor;	make all)
	
clean:
	@(cd bin;	make clean)
	@(cd UDP;	make clean)
	@(cd LiDAR;	make clean)
	@(cd processor;	make clean)

remove:
	sudo rm -rf ${INC_PATH}
	sudo rm ${LIB_PATH}/libKanavi*
	sudo rm ${PKG_PATH}/kanavi_lib.pc
