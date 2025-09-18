import matplotlib.pyplot as plt
import matplotlib.dates as mdates
from matplotlib import font_manager
from datetime import datetime, timedelta

# -----------------------------
# 1. 한글 폰트 자동 선택
# -----------------------------
def get_korean_font():
    korean_fonts = []
    for f in font_manager.findSystemFonts(fontpaths=None, fontext='ttf'):
        if any(name in f for name in ["NanumGothic", "NotoSansCJK", "UnDotum"]):
            korean_fonts.append(f)
    if korean_fonts:
        return font_manager.FontProperties(fname=korean_fonts[0])
    else:
        return None

font_prop = get_korean_font()
plt.rcParams['axes.unicode_minus'] = False  # 마이너스 깨짐 방지

# -----------------------------
# 2. 마일스톤과 세부 작업 정의
# -----------------------------
tasks = [
    {
        "milestone": "로봇 나비프라 이관",
        "start": "2025-09-15",
        "end": "2025-09-21",
        "subtasks": [
            {"name": "IPC OS 확인, 개발 환경 구성", "duration": 3},
            {"name": "모터 드라이버 초기 셋업/튜닝", "duration": 4},
        ]
    },
    {
        "milestone": "개발 기간",
        "start": "2025-09-15",
        "end": "2025-10-02",
        "subtasks": [
            {"name": "PLC 개발 지원", "duration": 5},
            {"name": "ACS 어댑터 지원", "duration": 5},
            {"name": "주행 데이터 취득", "duration": 7},
            {"name": "라인 셋업 교육", "duration": 3},
            {"name": "운영·유지보수 매뉴얼 수령", "duration": 2},
        ]
    },
    {
        "milestone": "주행부 완성 (주행 가능 상태)",
        "start": "2025-10-02",
        "end": "2025-10-02",
        "subtasks": [
            {"name": "삼성 2대 반입, 테스트/검증", "duration": 1},
            {"name": "P4H 어댑터 코드 제공", "duration": 1},
            {"name": "검수 및 문서 인수", "duration": 1},
        ]
    },
    {
        "milestone": "ACS 연동 테스트 (2대 반입)",
        "start": "2025-11-10",
        "end": "2025-11-10",
        "subtasks": [
            {"name": "광교 사무실, 이후 삼성 라인", "duration": 1}
        ]
    },
    {
        "milestone": "삼성 7대 반입",
        "start": "2025-12-26",
        "end": "2025-12-26",
        "subtasks": [
            {"name": "라인 설치 및 검수", "duration": 1},
            {"name": "문서 배포", "duration": 1}
        ]
    }
]

# -----------------------------
# 3. Gantt 차트 생성 (시각 개선)
# -----------------------------
fig, ax = plt.subplots(figsize=(12,7))
y_pos = 0
sub_height = 0.25
milestone_color = "#1f77b4"
subtask_color = "#aec7e8"

for task in tasks:
    start = datetime.strptime(task["start"], "%Y-%m-%d")
    end = datetime.strptime(task["end"], "%Y-%m-%d")
    # 마일스톤 막대
    ax.barh(y_pos, (end-start).days+1, left=start, height=0.5, color=milestone_color)
    # 마일스톤 텍스트
    if font_prop:
        ax.text(start - timedelta(days=0.5), y_pos, task["milestone"], va='center', ha='right', fontproperties=font_prop, fontsize=10, fontweight='bold')
    else:
        ax.text(start - timedelta(days=0.5), y_pos, task["milestone"], va='center', ha='right', fontsize=10, fontweight='bold')

    # 세부 작업 막대
    sub_y = y_pos - 0.35
    for sub in task["subtasks"]:
        ax.barh(sub_y, sub["duration"], left=start, height=sub_height, color=subtask_color)
        if font_prop:
            ax.text(start, sub_y, sub["name"], va='center', ha='left', fontproperties=font_prop, fontsize=9)
        else:
            ax.text(start, sub_y, sub["name"], va='center', ha='left', fontsize=9)
        # 완료일 강조 표시
        ax.plot(start + timedelta(days=sub["duration"]-1), sub_y, "o", color="red")
        sub_y -= 0.35

    # 마일스톤 구간 배경 음영
    ax.axhspan(sub_y+0.35, y_pos+0.25, facecolor="#f0f0f0", alpha=0.2)

    # 다음 마일스톤 y 위치
    y_pos = sub_y - 0.2

# y축 제거
ax.set_yticks([])
ax.xaxis.set_major_formatter(mdates.DateFormatter("%m-%d"))
ax.set_xlabel("날짜", fontproperties=font_prop if font_prop else None)
ax.set_title("P4H Gantt 차트", fontproperties=font_prop if font_prop else None)
plt.tight_layout()

# -----------------------------
# 4. PNG 저장
# -----------------------------
output_file = "gantt_chart_visual.png"
plt.savefig(output_file, dpi=150)
print(f"Gantt 차트가 '{output_file}'로 저장되었습니다.")
