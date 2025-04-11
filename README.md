# ROS1/ROS2 Hybrid Build Project Documentation

## 1. 프로젝트 개요

이 프로젝트는 ROS1과 ROS2를 하나의 소스 트리에서 동시에 빌드 및 실행할 수 있도록 구성된 하이브리드 구조입니다.
특정 LiDAR 모델(R2, R4, R270)에 따라 UDP로 데이터를 수신하고, ROS1 또는 ROS2 메시지로 퍼블리시합니다.

### 지원하는 OS

- 20.04에서 테스트 완료
- 그 외 버전은 확인 필요

---

## 2. 소스코드 트리 구조

```bash
directory/
└── src/
    └── kanavi_vl/
        ├── include/
        │   ├── argv_parser.hpp
        │   ├── common.h
        │   ├── kanavi_lidar.h
        │   ├── r270_spec.h
        │   ├── r2_spec.h
        │   ├── r4_spec.h
        │   ├── udp.h
        │   └── kanavi_vl/
        │       ├── ros1/
        │       │   └── kanavi_node.h
        │       └── ros2/
        │           └── kanavi_node.h
        ├── src/
        │   ├── lidar/
        │   │   ├── CMakeLists.txt
        │   │   └── kanavi_lidar.cpp
        │   ├── node_ros1/
        │   │   ├── CMakeLists.txt
        │   │   └── kanavi_node.cpp
        │   ├── node_ros2/
        │   │   ├── CMakeLists.txt
        │   │   └── kanavi_node.cpp
        │   ├── R2/
        │   │   └── main.cpp
        │   ├── R270/
        │   │   └── main.cpp
        │   ├── R4/
        │   │   └── main.cpp
        │   └── udp/
        │       ├── CMakeLists.txt
        │       └── udp.cpp
        ├── CMakeLists.txt
        └── package.xml
```

---

## 3. 파일별 기능 설명

### 루트 파일

- **CMakeLists.txt**: 전체 프로젝트의 빌드 설정
- **package.xml**: ROS 패키지 메타 정보 및 의존성 정의

### include/

- `argv_parser.hpp`: 커맨드라인 파라미터 파서
- `common.h`: 공통 매크로 및 타입 정의
- `kanavi_lidar.h`: LiDAR 처리 클래스 인터페이스
- `r2_spec.h`, `r4_spec.h`, `r270_spec.h`: 모델별 LiDAR 스펙 정의
- `udp.h`: UDP 통신 관련 정의
- `kanavi_node.h` (ros1/ros2): 각각의 ROS 버전에 따른 노드 정의

### src/

- **lidar/kanavi_lidar.cpp**: LiDAR 데이터 처리 구현
- **node_ros1/kanavi_node.cpp**: ROS1 노드 정의
- **node_ros2/kanavi_node.cpp**: ROS2 노드 정의
- **R2/R4/R270/main.cpp**: 모델별 실행 메인 파일
- **udp/udp.cpp**: UDP 통신 처리

---

## 4. 빌드 및 실행 방법

### ROS1 환경

```bash
catkin_make
rosrun kanavi_vl R4 -h
```

### ROS2 환경

```bash
colcon build
ros2 run kanavi_vl R4 -h
```

#### CLI 사용법

프로그램 실행 시 `-h` 옵션을 통해 다음과 같은 도움말이 출력됩니다:

```bash
[HELP]============
-i : set Network Information
    ex) -i [ip] [port]
-m : set multicast IP Address
    ex) -m [multicast ip]
-fix : set fixed frame Name for RViz
-topic : set topic name for RViz
```

##### 📌 파라미터 설명

| 파라미터 이름            | 설명                                     | 예시                             |
|--------------------------|------------------------------------------|----------------------------------|
| `-i`        | 네트워크 IP 및 포트 설정                 | `-i 192.168.0.1 8888`   |
| `-m`            | 멀티캐스트 IP 설정                       | `-m 224.0.0.1`   |
| `-fix`          | RViz에 사용될 fixed frame 이름 지정      | `-fix map`        |
| `-topic`                | ROS에서 퍼블리시할 topic 이름 지정       | `-topic /scan`                  |

> 참고: 파라미터 이름은 `KANAVI::ROS::PARAMETER_***` 상수로 관리됩니다.

### Run Node

#### ROS1

```bash
# R2
rosrun kanavi_vl R2 -i 192.168.123.100 5000 -m 224.0.0.5
# R270
rosrun kanavi_vl R270 -i 192.168.123.100 5000 -m 224.0.0.5 -fix map -topic kanavi_r270_msg
# R4
rosrun kanavi_vl R4 -i 192.168.123.100 5000 -m 224.0.0.5
```

#### ROS2

```bash
# R2
ros2 run kanavi_vl R2 -i 192.168.123.100 5000 -m 224.0.0.5
# R270
ros2 run kanavi_vl R270 -i 192.168.123.100 5000 -m 224.0.0.5 -fix map -topic kanavi_r270_msg
# R4
ros2 run kanavi_vl R4 -i 192.168.123.100 5000 -m 224.0.0.5
```

#### result

![ros1 R4](./images/ros1_r4.png)

![ros1 R270](./images/ros1_r270.png)

![ros2 R4](./images/ros2_r4.png)

![ros2 R270](./images/ros2_r270.png)

---

## 5. 라이선스 관련

본 프로젝트는 BSD 3-Clause License 하에 배포됩니다.  
루트 디렉토리에 `LICENSE` 파일이 포함되어 있습니다.

### 각 파일에 추가 권장되는 주석 (선택사항)

```cpp
// Copyright (c) 2024, Kanavi Mobility
// All rights reserved.
//
// This file is part of the ROS1/ROS2 Hybrid Build Project.
// Licensed under the BSD 3-Clause License.
// You may obtain a copy of the License at the root of this repository (LICENSE file).
```

---

## 6. CMakeLists.txt 및 package.xml 전체 설명

### 📄 CMakeLists.txt 전체 설명

`CMakeLists.txt`는 ROS1과 ROS2 환경에서 공통 소스 코드를 분기 처리하여 각각 빌드할 수 있도록 구성되어 있습니다.

#### 주요 역할

- 전체 프로젝트의 빌드 규칙 정의
- 하위 디렉토리 (`src/node_ros1`, `src/node_ros2`) 별로 모듈화
- ROS1/ROS2의 의존성 패키지 별도 설정

#### 핵심 구조

```cmake
cmake_minimum_required(VERSION 3.8)
project(kanavi_vl)

...

# ROS1/ROS2 버전 확인
if(DEFINED ENV{ROS_VERSION})
    if($ENV{ROS_VERSION} STREQUAL "1")  # ROS1
        # set(ROS1 TRUE)
        message(STATUS "Detected ROS1 environment")
        add_definitions(-DROS1)
        set(ROS_MACRO ROS1)  # ROS1 매크로
    elseif($ENV{ROS_VERSION} STREQUAL "2")  # ROS2
        # set(ROS2 TRUE)
        message(STATUS "Detected ROS2 environment")
        add_definitions(-DROS2)
        set(ROS_MACRO ROS2)  # ROS1 매크로
    else()
        message(FATAL_ERROR "Unsupported ROS_VERSION: $ENV{ROS_VERSION}")
    endif()
else()
    message(FATAL_ERROR "ROS_VERSION is not set. Please source your ROS environment.")
endif()

# ROS1으로 빌드
if($ENV{ROS_VERSION} STREQUAL "1")

 message("ROS1 BUILD")

 find_package(catkin REQUIRED COMPONENTS
 roscpp
 std_msgs
 pcl_conversions
 pcl_ros
 visualization_msgs
 )
 
 set(LIBRARIES
  kanavi_node
  kanavi_udp
  kanavi_lidar
 )

 foreach(LIBRARY ${LIBRARIES})
  set(LIB_OBJS ${LIB_OBJS} $<TARGET_OBJECTS:${LIBRARY}>)
 endforeach()

 add_library(kanavi_node
 src/node_ros1/kanavi_node.cpp)

 add_library(kanavi_udp
 src/udp/udp.cpp)

 add_library(kanavi_lidar
 src/lidar/kanavi_lidar.cpp)
 
###########
## Build ##
###########

 include_directories(
  include
  include/${PROJECT_NAME}
  include/${PROJECT_NAME}/ros1
  ${catkin_INCLUDE_DIRS}
 )

 #----define R270 node 
 add_executable(R270 
  src/R270/main.cpp
  ${LIB_OBJS}
 )

 target_link_libraries(R270
  kanavi_node
  kanavi_udp
  kanavi_lidar
  ${catkin_LIBRARIES}
 )

 #----define R4 node 
 add_executable(R4 
  src/R4/main.cpp
  ${LIB_OBJS}
 )

 target_link_libraries(R4
  kanavi_node
  kanavi_udp
  kanavi_lidar
  ${catkin_LIBRARIES}
 )
 
#############
## Install ##
#############

#--------------ROS2----------------------

elseif($ENV{ROS_VERSION} STREQUAL "2")
 message("ROS2 BUILD...")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(EIGEN eigen3 REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
 ament_cmake
 rclcpp
 pcl_conversions
 sensor_msgs
 std_msgs
)

# # set user defined Libs
set(LIBRARIES  
 kanavi_node
 kanavi_udp
 kanavi_lidar
)

foreach(LIBRARY ${LIBRARIES})
 set(LIB_OBJS ${LIB_OBJS} $<TARGET_OBJECTS:${LIBRARY}>)
endforeach()

include_directories(
 ${PCL_INCLUDE_DIRS}
 ${EIGEN_INCLUDE_DIRS}
 ${rclcpp_INCLUDE_DIRS}
 ${sensor_msgs_INCLUDE_DIRS}
 ${pcl_conversions_INCLUDE_DIRS}
 include
 include/${PROJECT_NAME}
)

# # add sub dir.
add_subdirectory(src/node_ros2)
add_subdirectory(src/udp)
add_subdirectory(src/lidar)

link_directories(
 ${PCL_LIBRARY_DIRS}
)

add_definitions(
 ${PCL_DEFINITIONS}
)

#----define R2 node 
add_executable(R2 
 src/R2/main.cpp
 ${LIB_OBJS}
)
ament_target_dependencies(R2 ${THIS_PACKAGE_INCLUDE_DEPENDS})
#-----------------------------------------------------------

#----define R4 node 
add_executable(R4 
 src/R4/main.cpp
 ${LIB_OBJS}
)
ament_target_dependencies(R4 ${THIS_PACKAGE_INCLUDE_DEPENDS})
#-----------------------------------------------------------

#----define R270 node 
add_executable(R270 
 src/R270/main.cpp
 ${LIB_OBJS}
)
ament_target_dependencies(R270 ${THIS_PACKAGE_INCLUDE_DEPENDS})

target_link_libraries(R270 
 ${PCL_LIBRARIES}
 ${EIGEN_LIBRARIES}
)
#-----------------------------------------------------------

install(TARGETS R2 R4 R270
  DESTINATION lib/${PROJECT_NAME})

ament_package()

endif()
```

---

### 📄 package.xml 전체 설명

`package.xml`은 이 패키지의 메타 정보와 ROS 의존성을 정의하는 파일입니다.

#### 주요 역할

- 이름, 버전, 설명, 라이선스 명시
- ROS 환경에서 필요한 메시지 및 실행 의존성 설정

#### 예시 구성

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>kanavi_vl</name>
  <version>0.0.1</version>
  <description>Hybrid ROS1/2 build support for Kanavi LiDAR</description>
 <maintainer email="twchong@kanavi-mobility.com">tw</maintainer>
  <license>BSD-3-Clause</license>

  <buildtool_depend condition="$ROS_VERSION == 1">catkin</buildtool_depend>
  <build_depend condition="$ROS_VERSION == 1">roscpp</build_depend>
  <build_depend condition="$ROS_VERSION == 1">std_msgs</build_depend>
  <build_export_depend condition="$ROS_VERSION == 1">roscpp</build_export_depend>
  <build_export_depend condition="$ROS_VERSION == 1">std_msgs</build_export_depend>
  <exec_depend condition="$ROS_VERSION == 1">roscpp</exec_depend>
  <exec_depend condition="$ROS_VERSION == 1">std_msgs</exec_depend>

  <buildtool_depend condition="$ROS_VERSION == 2">ament_cmake</buildtool_depend>

  <depend condition="$ROS_VERSION == 2">rclcpp</depend>
  <depend condition="$ROS_VERSION == 2">std_msgs</depend>
  <depend condition="$ROS_VERSION == 2">sensor_msgs</depend>
  <depend condition="$ROS_VERSION == 2">pcl_conversions</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type condition="$ROS_VERSION == 1">catkin</build_type>
    <build_type condition="$ROS_VERSION == 2">ament_cmake</build_type>
  </export>
</package>
```

build_depend condition="$ROS_VERSION == ?"을 통해 현재 ROS 버전을 확인하고 ROS 빌드시스템(`catkin_make`, `colcon build`)이 패키지를 올바르게 처리할 수 있게 해줍니다.

---

## 7. main.cpp

```cpp
#include <r270_spec.h>

#if defined(ROS1)
#include <ros1/kanavi_node.h>

int main(int argc, char **argv)
{
	printf("ROS1 build test\n");
	ros::init(argc, argv, "r270");

	kanavi_node node("r270", argc, argv);

	node.run();

	return 0;
}

#elif defined (ROS2)

#include <ros2/kanavi_node.h>

int main(int argc, char **argv)
{
	// init ROS2
	rclcpp::init(argc, argv);

	// generate node
	auto node = std::make_shared<kanavi_node>("r270", argc, argv);

	// start node
	rclcpp::spin(node);

	// exit node
	rclcpp::shutdown();

	return 0;
}

#endif

```

---
