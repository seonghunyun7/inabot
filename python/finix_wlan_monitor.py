import subprocess
import time
import matplotlib.pyplot as plt
from collections import deque

#robot1
#wifi : 172.29.66.239
#pc: 172.29.66.245

#robot2
#wifi : 172.29.66.240
#pc: 172.29.66.246

#robot3
#wifi: 172.29.66.241
#pc: 172.29.66.247

#robot4
#wifi : 172.29.66.242
#pc:172.29.66.248

#robot5
#wifi : 172.29.66.243
#pc : 172.29.66.249

#robot6
#wifi : 172.29.66.244
#pc: 172.29.66.250
#lr1234!

#시점	Signal Strength (dBm)	Link Quality (%)	현상
#평상시	-40 ~ -60	70% ~ 100%	안정                연결 상태
#DFS 스캔 발생	-90 ~ -100	0% ~ 10%	            재연결 중 (Scanning...)
#이후 회복	-40 ~ -60	70% ~ 100%	                다시 연결됨, 안정화
#Wi-Fi 표준	최대 Bit Rate
#802.11b	11 Mb/s
#802.11g	54 Mb/s
#802.11n	~600 Mb/s
#802.11ac	~1.3 Gb/s 이상
#802.11ax	~9.6 Gb/s
#Bit Rate	현재 Wi-Fi 링크가 사용하는 전송 속도. 단위는 Mb/s (메가비트/초)
#측정 기준	실제 데이터 처리량이 아닌, 이론적 최대 속도 또는 현재 연결 상태에서 허용된 속도
#영향 요소	- 신호 세기 (dBm)
#- 링크 품질 (Link Quality)
#- 간섭(Interference)
#- 거리
#- Wi-Fi 표준 (802.11n/ac/ax 등)
#- 채널 혼잡도
#변동 가능	예, 환경 변화나 간섭, DFS 재스캔 등으로 인해 Bit Rate는 실시간으로 변할 수 있음

#Wi-Fi 신호 세기(dBm)
#Link Quality (%)
#Bit Rate (Mb/s)
#DFS 스캔 또는 재접속 징후 감지 (신호가 매우 약하고 Link Quality도 낮을 때 경고)
#실시간 그래프 시각화

# Wi-Fi 인터페이스 자동 탐지
def detect_wifi_interface():
    try:
        result = subprocess.run(['iwconfig'], stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=True, universal_newlines=True)
        for line in result.stdout.splitlines():
            if 'IEEE 802.11' in line:
                return line.split()[0]
    except subprocess.CalledProcessError:
        pass
    return None

# Signal Strength (dBm)
def get_signal_strength(interface):
    try:
        result = subprocess.run(['iwconfig', interface], stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=True, universal_newlines=True)
        for line in result.stdout.split('\n'):
            if 'Signal level=' in line:
                signal_level = line.split('Signal level=')[1].split(' ')[0]
                return int(signal_level)
    except subprocess.CalledProcessError:
        pass
    return None

# Link Quality (%)
def get_link_quality(interface):
    try:
        result = subprocess.run(['iwconfig', interface], stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=True, universal_newlines=True)
        for line in result.stdout.split('\n'):
            if 'Link Quality=' in line:
                part = line.split('Link Quality=')[1].split(' ')[0]
                if '/' in part:
                    value, total = map(int, part.split('/'))
                    return round((value / total) * 100, 1)
    except subprocess.CalledProcessError:
        pass
    return None

# Bit Rate (Mb/s)
def get_bit_rate(interface):
    try:
        result = subprocess.run(['iwconfig', interface], stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=True, universal_newlines=True)
        for line in result.stdout.split('\n'):
            if 'Bit Rate=' in line:
                part = line.split('Bit Rate=')[1].split(' ')[0]
                return float(part)
    except subprocess.CalledProcessError:
        pass
    return None

# 실시간 모니터링 및 시각화
def monitor_and_plot():
    interface = detect_wifi_interface()
    if not interface:
        print("No wireless interface found.")
        return

    print(f"Detected Wi-Fi interface: {interface}")

    max_points = 30
    signal_history = deque(maxlen=max_points)
    quality_history = deque(maxlen=max_points)
    bitrate_history = deque(maxlen=max_points)
    time_history = deque(maxlen=max_points)
    scan_events = []

    plt.ion()
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 9))

    # Signal Strength plot
    line_signal, = ax1.plot([], [], label="Signal Strength (dBm)", color='tab:blue')
    ax1.set_ylim(-100, -20)
    ax1.set_ylabel("dBm")
    ax1.set_title("Wi-Fi Signal Strength")
    ax1.grid(True)
    ax1.legend()

    # Link Quality plot
    line_quality, = ax2.plot([], [], label="Link Quality (%)", color='tab:green')
    ax2.set_ylim(0, 100)
    ax2.set_ylabel("%")
    ax2.set_title("Wi-Fi Link Quality")
    ax2.grid(True)
    ax2.legend()

    # Bit Rate plot
    line_bitrate, = ax3.plot([], [], label="Bit Rate (Mb/s)", color='tab:orange')
    ax3.set_ylim(0, 1000)
    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel("Mb/s")
    ax3.set_title("Wi-Fi Bit Rate")
    ax3.grid(True)
    ax3.legend()

    t = 0

    while True:
        signal = get_signal_strength(interface)
        quality = get_link_quality(interface)
        bit_rate = get_bit_rate(interface)
        timestamp = time.strftime('%H:%M:%S')

        print(f"[{timestamp}] Signal: {signal if signal is not None else 'N/A'} dBm | "
              f"Link Quality: {quality if quality is not None else 'N/A'} % | "
              f"Bit Rate: {bit_rate if bit_rate is not None else 'N/A'} Mb/s")

        # DFS 재접속 감지
        if signal is not None and signal <= -90 and quality is not None and quality < 10:
            print(f"⚠️  [{timestamp}] DFS Scan or Disconnection Detected!")
            scan_events.append(t)

        # 그래프 데이터 저장
        signal_history.append(signal if signal is not None else -100)
        quality_history.append(quality if quality is not None else 0)
        bitrate_history.append(bit_rate if bit_rate is not None else 0)
        time_history.append(t)

        # 그래프 데이터 설정
        line_signal.set_data(time_history, signal_history)
        line_quality.set_data(time_history, quality_history)
        line_bitrate.set_data(time_history, bitrate_history)

        # x축 설정
        for ax in (ax1, ax2, ax3):
            ax.set_xlim(max(0, t - max_points), t + 1)

        # DFS 감지 마커 표시
        for event_time in scan_events:
            for ax in (ax1, ax2, ax3):
                ax.axvline(x=event_time, color='red', linestyle='--', alpha=0.5)

        # 그래프 그리기
        fig.tight_layout()
        fig.canvas.draw()
        fig.canvas.flush_events()

        time.sleep(5)
        t += 5

if __name__ == '__main__':
    monitor_and_plot()