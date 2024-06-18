ROS_PATH=~/catkin_ws
INC_PATH=${ROS_PATH}/src
BASH_STR=catkin_ws/devel/setup.bash

LIB_PATH=../lib

CHECK_RESULT=$(shell cat ~/.bashrc | grep catkin_ws/devel/setup.bash)
VAR=$(words $(findstring ${BASH_STR}, ${CHECK_RESULT}))

ldflags=$(shell pkg-config --libs kanavi_lib)
check_LIB=true

#----os check------
OS_NAME=$(OS)
UNAME_S:= $(shell uname -s)

ifeq ( $(OS), WIndows_NT )
	OS_NAME="windows"
else
ifeq (${UNAME_S},Linux)
	OS_NAME="linux"
endif
ifeq ($(UNAME_S),Darwin)
	OS_NAME="mac"
endif
endif

all: os_check lib ros

os_check:
#----os check--------
	echo "OS Checking..."
ifneq ($(OS_NAME), "linux")
	echo "${OS_NAME} not provide..."
	exit 1
endif

lib:
	@echo "Check KANAVI LIB..."

# ifneq ("$(ldflags)", "")
# 	@echo "Lib already set"
# 	@check_LIB=false
# else
# 	@echo "Library not installed"
# 	@check_LIB=true
# endif

# ifeq ( $(check_LIB), true )
#	(cd ${LIB_PATH}; make clean; make all)
# endif
	(cd ${LIB_PATH}; make remove; make clean; make install)

ros:
	@echo "copy Carnvavicom lidar ros"
	mkdir -p ${INC_PATH}
	cp -rpf kanavilidar_ros ${ROS_PATH}/src
	(cd ${ROS_PATH};	catkin_make)
	sudo ldconfig
ifeq (${VAR},1)
	@echo "Already Set"
else 
	echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
endif
