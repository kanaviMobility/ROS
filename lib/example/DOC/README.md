# Kanavi-Mobility LIBS

## LIB compile

```sh
cd lib
make install	# /usr/lib/KANAVI_SDK, /usr/include/KANAVI_SDK에 copy
```

## exameple 구동

### OPENCV 사용 시

```sh
# KANAVI_SDK/example/src/Makefile 수정
OPENCV_PATH, OPENCV_LIB 활성화

# main.cpp
#define OPENCV_ 활성화
```

### OPENCV 미사용 시

```sh
# KANAVI_SDK/example/src/Makefile 수정
OPENCV_PATH, OPENCV_LIB 비활성화

# main.cpp
#define OPENCV_ 비활성화
```

### 구동

```sh
./test_app -i [ip] [port] -m(멀티캐스트)
./test_app -i 192.168.123.99 5000 # VL-AS16
./test_app -i 192.168.123.99 5000 -m # R2
```

- VL-AS16은 기본적 unicast로 동작
- R2는 기본적으로 multicast로 동작
