#pragma once

//임시 파일임.. 나중에 libpcan 설치 할 거임.
typedef unsigned char BYTE;
typedef unsigned long DWORD;

typedef struct {
    DWORD ID;            // 11/29비트 CAN ID
    BYTE  MSGTYPE;       // 메시지 타입 (표준/확장/에러)
    BYTE  LEN;           // 데이터 길이
    BYTE  DATA[8];       // 실제 데이터
} TPCANMsg_;
