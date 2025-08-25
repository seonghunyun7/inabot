Inabot Logger 사용법
개요

Inabot::Logger는 Boost.Log를 기반으로 한 C++ 로깅 유틸리티입니다.

    콘솔과 파일 동시 출력

    컬러 출력 지원

    로그 레벨별 필터링 가능

    에러 로그는 별도 파일 저장

설치 및 준비

    Boost 라이브러리 설치

        Ubuntu 예시:

        sudo apt install libboost-all-dev

    logger.h와 logger.cpp를 프로젝트에 포함

    빌드 시 Boost.Log 라이브러리 연결

        예) -lboost_log -lboost_system -lboost_thread -lpthread

사용법
1. 헤더 포함

#include <utils/logger.h>

2. 네임스페이스

using namespace Inabot;

3. 로거 초기화 (보통 자동 초기화됨)

Logger::get();

4. 로그 레벨 설정 (필요 시)

Logger::get().SetSeverityMin(severity_level::debug);  // 최소 로그 레벨 debug 이상 출력

5. 로그 출력

    매크로 사용 (가장 간편)

LOG_INFO("프로그램 시작, 버전: %s", version.c_str());
LOG_WARNING("디스크 공간 부족: %.2f%%", free_space);
LOG_ERROR("서버 연결 실패: %s", server_address.c_str());

    스트림 스타일 사용

NLOG(info) << "Inabot 초기화 완료";
NLOG(error) << "파일 열기 실패: " << fileName;

로그 파일 경로

    기본 로그 파일:
    ~/.ros/logs/<프로세스명>/<프로세스명>.log

    에러 로그 파일:
    ~/.ros/logs/<프로세스명>/error/error.log

예시

#include <utils/logger.h>

int main()
{
    using namespace Inabot;

    Logger::get().SetSeverityMin(severity_level::debug);

    LOG_INFO("프로그램 시작");

    int count = 10;
    NLOG(debug) << "카운트 값: " << count;

    LOG_ERROR("네트워크 연결 실패");

    return 0;
}

참고

    로그 메시지는 날짜, 시간, 로그 레벨, 실행시간, 소스 파일명, 라인 번호와 함께 출력됩니다.

    콘솔 출력 시 로그 레벨에 따라 색상이 적용됩니다.

    로그 폴더가 없으면 수동으로 생성해 주세요.


# SystemStatusParser

`SystemStatusParser` 클래스는 리눅스 기반 임베디드 또는 로봇 PC 시스템에서 실시간으로 시스템 상태를 수집, 파싱하여 ROS 메시지로 제공하는 유틸리티입니다.

---

## 주요 기능

- **CPU 사용률 측정**  
  `/proc/stat` 파일을 읽어 이전과 현재 CPU 시간 데이터를 비교하여 CPU 사용률(%)을 계산합니다.

- **CPU 온도 측정**  
  `/sys/class/thermal/thermal_zone0/temp`에서 CPU 온도를 읽어 섭씨 단위로 반환합니다.

- **메모리 사용량 확인**  
  `/proc/meminfo`에서 전체 메모리와 사용 중인 메모리(`MemTotal`, `MemFree`, `Buffers`, `Cached`)를 파싱하여 메가바이트 단위로 제공합니다.

- **디스크 I/O 통계 수집**  
  `/proc/diskstats`에서 특정 디바이스(예: `sda` 또는 `nvme0n1`)의 섹터 읽기/쓰기량을 읽어, 이전 측정치와 비교하여 읽기/쓰기 속도를 메가바이트 단위로 계산합니다.

- **상위 프로세스 정보 제공**  
  `/proc` 내 프로세스별 CPU 시간 및 메모리 사용량을 분석하여 CPU 및 메모리 사용량 기준 상위 3개 프로세스의 이름과 사용량을 반환합니다.

- **ROS 메시지 형태로 결과 반환**  
  `lrbot2_msgs::msg::SystemStatus` 메시지 타입으로 시스템 상태 데이터를 통합하여 반환, ROS 환경 내에서 편리하게 활용할 수 있습니다.

---

## 사용법

1. **클래스 생성 및 초기화**

```cpp
SystemStatusParser status_parser;

    상태 업데이트 및 메시지 획득

lrbot2_msgs::msg::SystemStatus status_msg = status_parser.getSystemStatus();

    ROS 노드에서 주기적으로 호출하여 상태 모니터링 가능

참고 사항

    디스크 이름은 시스템에 따라 다를 수 있습니다. (예: sda, nvme0n1 등)
    실제 환경에 맞는 디스크 이름으로 설정해야 합니다.
     - lsblk
     - DiskStats curr_disk_stats = readDiskStats("nvme0n1");

    CPU 온도 경로(/sys/class/thermal/thermal_zone0/temp)가 환경에 따라 다를 수 있으므로, 필요한 경우 경로를 조정하세요.

    ROS 메시지 타입 lrbot2_msgs::msg::SystemStatus는 별도 정의된 메시지로, 프로젝트 환경에 맞게 커스텀 메시지를 정의해야 합니다.

예시 출력 (ROS 메시지 필드 예시)
필드	설명	단위
cpu_usage_percent	CPU 사용률	%
cpu_temperature	CPU 온도	℃
mem_total_mb	전체 메모리	MB
mem_used_mb	사용 중인 메모리	MB
disk_read_mb_per_sec	디스크 읽기 속도	MB/s
disk_write_mb_per_sec	디스크 쓰기 속도	MB/s
top_cpu_processes	CPU 사용량 상위 3개 프로세스명 및 사용률	-
top_mem_processes	메모리 사용량 상위 3개 프로세스명 및 사용량	MB

yoon@yoon-ThinkPad-X1-Carbon-Gen-12:~$ ros2 topic echo /system_status
cpu_use: 5.299999713897705
cpu_temp: 63.0
mem_total: 31599.82421875
mem_used: 6923.26953125
cpu_top_processes:
- gnome-s+
- ibus-da+
- gnome-t+
- ibus-ex+
- code
cpu_top_usages:
- 512.5
- 12.5
- 12.5
- 6.199999809265137
- 6.199999809265137
mem_top_processes:
- code
- Isolate+
- gnome-s+
- firefox
- code
mem_top_usages:
- 2.700000047683716
- 2.200000047683716
- 2.200000047683716
- 2.0999999046325684
- 1.7000000476837158
disk_read: 0.00 KB/s
disk_write: 197092.00 KB/s