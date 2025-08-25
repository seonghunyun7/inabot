📦 serial 라이브러리 (ROS 2용) 빌드 및 설치 가이드


✅ 1. 소스 클론 및 브랜치 생성

git clone https://github.com/wjwwood/serial.git
cd serial
git checkout -b pure

    main 브랜치는 ROS 1용 (catkin 기반)이므로, ROS 2에서는 에러가 발생합니다. 이를 피하기 위해 로컬 브랜치를 생성하여 수정합니다.

✅ 2. CMakeLists.txt 수정

serial 루트 디렉터리의 CMakeLists.txt 파일 내용을 아래처럼 변경합니다 (catkin 관련 코드 전부 제거):

cmake_minimum_required(VERSION 3.5)
project(serial)

set(CMAKE_CXX_STANDARD 14)

include_directories(include)

set(SOURCES
    src/serial.cc
    include/serial/serial.h
    include/serial/v8stdint.h
)

if(APPLE)
    list(APPEND SOURCES src/impl/unix.cc src/impl/list_ports/list_ports_osx.cc)
    find_library(FOUNDATION_LIBRARY Foundation)
    find_library(IOKIT_LIBRARY IOKit)
elseif(UNIX)
    list(APPEND SOURCES src/impl/unix.cc src/impl/list_ports/list_ports_linux.cc)
else()
    list(APPEND SOURCES src/impl/win.cc src/impl/list_ports/list_ports_win.cc)
endif()

add_library(serial STATIC ${SOURCES})

if(APPLE)
    target_link_libraries(serial ${FOUNDATION_LIBRARY} ${IOKIT_LIBRARY})
elseif(UNIX)
    target_link_libraries(serial pthread rt)
else()
    target_link_libraries(serial setupapi)
endif()

install(TARGETS serial
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/serial DESTINATION include)

✅ 3. 빌드 및 설치

mkdir -p build && cd build
cmake ..
make
sudo make install

📁 설치 결과
항목	경로
헤더 파일	/usr/local/include/serial
정적 라이브러리	/usr/local/lib/libserial.a
✅ 4. ROS 2 패키지 연동 방법

CMakeLists.txt에 다음 내용을 추가하여 연동합니다:

# Include serial headers
include_directories(/usr/local/include)

# Find the serial library
find_library(SERIAL_LIBRARY
  NAMES serial
  PATHS /usr/local/lib
  REQUIRED
)

# Link the serial library to your node
target_link_libraries(your_node_name
  ${SERIAL_LIBRARY}
)

💡 참고

    serial::Serial 클래스를 사용하여 시리얼 통신을 수행할 수 있습니다.

    ROS 2 노드에서 다음처럼 사용합니다:

#include <serial/serial.h>

serial::Serial imu_serial("/dev/ttyUSB0", 115200, serial::Timeout::simpleTimeout(1000));

✨ 요약 정리
항목	내용
기본 브랜치	main → ROS 1용 (catkin)
수정 브랜치	pure → catkin 제거, CMake 순수화
설치 위치	/usr/local/include/serial, /usr/local/lib/libserial.a
ROS 2 연동 방법	find_library, include_directories 활용


ls /usr/local/include/serial
ls /usr/local/lib/libserial.*


==

✅ (2) libserial (crayzeewulf)로 바꾸고 싶을 경우:

    git clone:

git clone https://github.com/crayzeewulf/libserial.git
cd libserial
mkdir build && cd build
cmake ..
make
sudo make install

이 경우 기존 #include <libserial/serial.h> 그대로 사용 가능

CMake 설정은 /usr/local/include로 이미 잡혀 있을 가능성 높음