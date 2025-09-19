#!/usr/bin/env python3
import socket
import os
import time

# OTA 서버 정보
SERVER_HOST = '127.0.0.1'
SERVER_PORT = 5005
FILE_TO_SEND = '/home/ysh/ota/ota_update.tar.gz'

if not os.path.exists(FILE_TO_SEND):
    print(f"File not found: {FILE_TO_SEND}")
    exit(1)

file_size = os.path.getsize(FILE_TO_SEND)
print(f"File size: {file_size} bytes")

sent_bytes = 0
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    print(f"Connecting to OTA server {SERVER_HOST}:{SERVER_PORT} ...")
    try:
        s.connect((SERVER_HOST, SERVER_PORT))
    except ConnectionRefusedError:
        print("Connection failed! Make sure OTA node is running.")
        exit(1)
    print("Connected.")

    # 파일 전송
    with open(FILE_TO_SEND, 'rb') as f:
        while True:
            chunk = f.read(4096)
            if not chunk:
                break
            s.sendall(chunk)
            sent_bytes += len(chunk)

    # 송신 종료 신호
    s.shutdown(socket.SHUT_WR)

print(f"File sent successfully! Total bytes sent: {sent_bytes}")
if sent_bytes == file_size:
    print("✔ All bytes sent correctly")
else:
    print(f"⚠ Mismatch: sent {sent_bytes} bytes, expected {file_size} bytes")
