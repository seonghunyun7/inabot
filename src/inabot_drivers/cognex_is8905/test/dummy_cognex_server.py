# dummy_cognex_server.py
import socket
import time

HOST = '127.0.0.1'
PORT = 5000       # Cognex_IS8905 노드와 동일한 포트

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((HOST, PORT))
    s.listen(1)
    print(f"Dummy Cognex server listening on {HOST}:{PORT}")
    
    conn, addr = s.accept()
    with conn:
        print(f"Connected by {addr}")
        while True:
            try:
                # 1을 문자열로 보내고 개행 문자 제거
                conn.sendall(b"1")
                time.sleep(0.1)  # 100ms마다 전송
            except BrokenPipeError:
                print("Client disconnected")
                break
