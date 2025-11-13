"""
HSU - Smart : Load Cell Robot Simulation (State-Based Control)
로드셀 기반 카트 제어 시뮬레이션 (Python)
C 코드의 '이산 상태' 계산법 적용 버전
"""

import pygame
import serial
import serial.tools.list_ports
import math
import time
from collections import deque
import sys

# 초기화
pygame.init()

# 화면 설정
WINDOW_WIDTH = 1400
WINDOW_HEIGHT = 800
screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
pygame.display.set_caption("HSU-Future Mobility! (RPM-Based)")

# 색상 정의 (이전과 동일)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GRAY = (200, 200, 200)
DARK_GRAY = (100, 100, 100)
RED = (239, 68, 68)
GREEN = (16, 185, 129)
BLUE = (59, 130, 246)
PURPLE = (168, 85, 247)
YELLOW = (251, 191, 36)
LIGHT_BLUE = (147, 197, 253)

# 한글 지원 폰트 설정 (이전과 동일)
def get_korean_font(size):
    """한글을 지원하는 시스템 폰트 찾기"""
    font_candidates = [
        'malgungothic',  # Windows
        'malgun gothic',
        'nanumgothic',  # Linux
        'applegothic',  # macOS
        'arial',
    ]
    
    for font_name in font_candidates:
        try:
            return pygame.font.SysFont(font_name, size)
        except:
            continue
    
    # 모두 실패하면 기본 폰트
    return pygame.font.Font(None, size)

# 폰트
font_large = get_korean_font(36)
font_medium = get_korean_font(28)
font_small = get_korean_font(20)

class RobotSimulation:
    def __init__(self):
        # 로드셀 입력
        self.f_left = 0.0  # 초기값 0으로 변경
        self.f_right = 0.0 # 초기값 0으로 변경
        self.input_mode = 'manual'  # 'manual' or 'arduino'
        
        # 아두이노 연결 (이전과 동일)
        self.serial_port = None
        self.is_connected = False
        self.serial_buffer = ""
        self.serial_status = "Arduino 연결 안됨 (C키로 연결)"
        # --- (신규) 실제 모터 상태 변수 추가 ---
        self.L_REAL_RPM = 0.0
        self.R_REAL_RPM = 0.0
        self.L_POS = 0
        self.R_POS = 0
        self.L_TRQ = 0
        self.R_TRQ = 0
        
        # 로봇 파라미터
        self.mass = 10.0  # kg (이 로직에서는 사용되지 않음)
        self.wheel_base = 0.5  # m (좌우 회전 계산 시 필요)
        
        # 시뮬레이션 상태
        self.is_running = False # 물리 시뮬레이션 작동 여부
        self.simulation_mode = False # 'S'키로 제어하는 모드
        self.robot_x = 250.0
        self.robot_y = 250.0
        self.robot_angle = 0.0
        self.trajectory = deque(maxlen=200)
        
        # 시간
        self.last_time = time.time()
        
        # --- (신규) 아두이노로부터 받은 RPM 및 변환 계수 ---
        self.L_RPM_from_arduino = 0.0
        self.R_RPM_from_arduino = 0.0
        
        # RPM을 m/s로 변환하는 계수 (튜닝 필요)
        # (예: Arduino 50 RPM -> Python 0.15 m/s => 0.15/50 = 0.003)
        self.RPM_TO_MS_FACTOR = 0.0031
        
        # --- (변경) L/R Velocity는 이제 RPM 변환 결과임 ---
        self.Lvelocity = 0.0 # 왼쪽 바퀴 최종 속도 (m/s)
        self.Rvelocity = 0.0 # 오른쪽 바퀴 최종 속도 (m/s)
        
        # (참고) UI 표시에만 사용
        self.velocity = 0.0      # (V)
        self.angular_vel = 0.0   # (W)


    # --- 시뮬레이션 업데이트 (로직 변경) ---
    def update(self, dt):
        """시뮬레이션 업데이트 (아두이노 RPM 기반)"""
        
        # self.is_running은 self.simulation_mode에 의해 제어됨.
        if not self.is_running:
            # RPM 0으로 설정하여 UI에 반영
            self.Lvelocity = 0.0
            self.Rvelocity = 0.0
            self.velocity = 0.0
            self.angular_vel = 0.0
            return
        
        
        # 1. 아두이노에서 받은 RPM을 m/s (L/R velocity)로 변환
        # 아두이노 모터 모델(왼쪽 반전)을 Python의 차동구동 모델로 변환
        # Lvelocity (Python) = -L_RPM (Arduino) * Factor
        # Rvelocity (Python) =  R_RPM (Arduino) * Factor
        self.Lvelocity = -self.L_RPM_from_arduino * self.RPM_TO_MS_FACTOR
        self.Rvelocity = self.R_RPM_from_arduino * self.RPM_TO_MS_FACTOR
        
        # 3. 차동 구동 로봇의 V(선속도)와 W(각속도) 계산
        # V = (R + L) / 2
        # W = (R - L) / wheel_base
        V = (self.Rvelocity + self.Lvelocity) / 2.0
        W = (self.Rvelocity - self.Lvelocity) / self.wheel_base
        
        # (UI 표시용)
        self.velocity = V
        self.angular_vel = W
        
        scale = 100  # 화면 스케일
        
        # 4. 로봇 위치 및 각도 업데이트 (이전과 동일)
        self.robot_angle += W * dt
        self.robot_x += V * math.cos(self.robot_angle) * dt * scale
        self.robot_y += V * math.sin(self.robot_angle) * dt * scale
        
        # 경계 처리 (이전과 동일)
        if self.robot_x > 500:
            self.robot_x = 0
        elif self.robot_x < 0:
            self.robot_x = 500
        
        if self.robot_y > 500:
            self.robot_y = 0
        elif self.robot_y < 0:
            self.robot_y = 500
        
        # 경로 추가 (이전과 동일)
        self.trajectory.append((self.robot_x + 50, self.robot_y + 150))
    
    # --- 아두이노 함수 (신규 함수 추가) ---
    def connect_arduino(self):
        """아두이노 연결"""
        try:
            ports = list(serial.tools.list_ports.comports())
            if not ports:
                self.serial_status = "사용 가능한 포트 없음"
                return
            
            port_name = ports[0].device
            self.serial_port = serial.Serial(port_name, 115200, timeout=0.1) # 9600 Baud
            self.is_connected = True
            self.input_mode = 'arduino'
            self.serial_status = f"연결됨: {port_name}"
            time.sleep(2)  # 아두이노 리셋 대기
            
        except Exception as e:
            self.serial_status = f"연결 오류: {str(e)}"
    
    def disconnect_arduino(self):
        """아두이노 연결 해제"""
        if self.serial_port:
            if self.simulation_mode:
                # 연결을 끊기 전에, Arduino가 실제 모드로 돌아가도록 시도
                # (실패할 수도 있지만, 시도하는 것이 좋음)
                self.send_arduino_command('R')
            self.serial_port.close()
            self.serial_port = None
        
        self.is_connected = False
        self.input_mode = 'manual'
        self.serial_status = "연결 해제됨"
        
        # 연결이 끊기면 시뮬레이션 모드 강제 종료
        if self.simulation_mode:
            self.simulation_mode = False
            self.is_running = False
    
    def read_arduino_data(self):
        """아두이노 데이터 읽기"""
        if not self.is_connected or not self.serial_port:
            return
        
        try:
            if self.serial_port.in_waiting > 0:
                data = self.serial_port.readline().decode('utf-8').strip()
                # print(data) # 디버깅용
                if data:
                    self.parse_arduino_data(data)
        except Exception as e:
            self.serial_status = f"읽기 오류: {str(e)}"
    
    def parse_arduino_data(self, data):
        """아두이노 데이터 파싱 확장"""
        try:
            parts = data.split(',')
            left, right, l_rpm, r_rpm = None, None, None, None
            
            # 파싱 루프
            for part in parts:
                if part.startswith('L:'):
                    left = float(part.split(':')[1])
                elif part.startswith('R:'):
                    right = float(part.split(':')[1])
                elif part.startswith('L_RPM:'):
                    l_rpm = float(part.split(':')[1])
                elif part.startswith('R_RPM:'):
                    r_rpm = float(part.split(':')[1])
                    
                # (신규) 추가 데이터 파싱
                elif part.startswith('L_REAL:'):
                    self.L_REAL_RPM = float(part.split(':')[1])
                elif part.startswith('R_REAL:'):
                    self.R_REAL_RPM = float(part.split(':')[1])
                elif part.startswith('L_POS:'):
                    self.L_POS = int(part.split(':')[1])
                elif part.startswith('R_POS:'):
                    self.R_POS = int(part.split(':')[1])
                elif part.startswith('L_TRQ:'):
                    self.L_TRQ = int(part.split(':')[1])
                elif part.startswith('R_TRQ:'):
                    self.R_TRQ = int(part.split(':')[1])
            
            # 데이터 업데이트
            if left is not None and right is not None:
                self.f_left = left 
                self.f_right = right
            
            if l_rpm is not None and r_rpm is not None:
                self.L_RPM_from_arduino = l_rpm
                self.R_RPM_from_arduino = r_rpm

            # 상태창 텍스트 업데이트 (간략 정보)
            if all(v is not None for v in [left, right, l_rpm, r_rpm]):
                self.serial_status = f"CMD: L={l_rpm:.0f}/R={r_rpm:.0f}, ACT: L={self.L_REAL_RPM:.0f}/R={self.R_REAL_RPM:.0f}"

        except Exception as e:
            pass

    def send_arduino_command(self, cmd):
        """아두이노로 시리얼 명령 전송 ('S' 또는 'R')"""
        if self.is_connected and self.serial_port:
            try:
                self.serial_port.write(cmd.encode('utf-8'))
            except Exception as e:
                self.serial_status = f"쓰기 오류: {str(e)}"

    def toggle_simulation_mode(self):
        """'S' 키로 시뮬레이션 모드 토글"""
        if not self.is_connected:
            self.serial_status = "Arduino 연결이 필요합니다."
            return
            
        self.simulation_mode = not self.simulation_mode
        
        if self.simulation_mode:
            self.send_arduino_command('S') # Arduino: "시뮬레이션 시작 (모터 제어 중지)"
            self.is_running = True
            print("Simulation Mode: ON (Arduino 모터 제어 중지)")
        else:
            self.send_arduino_command('R') # Arduino: "실제 모드 재개 (모터 제어 시작)"
            self.is_running = False
            print("Simulation Mode: OFF (Arduino 모터 제어 재개)")
    
    def reset(self):
        """시뮬레이션 리셋"""
        # 시뮬레이션 모드였다면, Arduino를 실제 모드로 복귀시킴
        if self.simulation_mode:
            self.send_arduino_command('R')
            self.simulation_mode = False

        self.is_running = False
        self.robot_x = 250.0
        self.robot_y = 250.0
        self.robot_angle = 0.0
        self.velocity = 0.0
        self.angular_vel = 0.0
        self.trajectory.clear()
        
        # 상태 리셋
        self.Lvelocity = 0.0
        self.Rvelocity = 0.0
        self.f_left = 0.0
        self.f_right = 0.0
        self.L_RPM_from_arduino = 0.0
        self.R_RPM_from_arduino = 0.0
        
    # --- 그리기 함수 (draw_canvas는 이전과 동일) ---
    def draw_canvas(self, surface, offset_x, offset_y):
        """시뮬레이션 캔버스 그리기"""
        canvas_rect = pygame.Rect(offset_x, offset_y, 500, 500)
        pygame.draw.rect(surface, WHITE, canvas_rect)
        pygame.draw.rect(surface, DARK_GRAY, canvas_rect, 2)
        
        # 그리드
        for i in range(0, 501, 50):
            pygame.draw.line(surface, GRAY, 
                           (offset_x + i, offset_y), 
                           (offset_x + i, offset_y + 500), 1)
            pygame.draw.line(surface, GRAY, 
                           (offset_x, offset_y + i), 
                           (offset_x + 500, offset_y + i), 1)
        
        # 경로 그리기
        if len(self.trajectory) > 1:
            points = [(p[0] + offset_x - 50, p[1] + offset_y - 150) for p in self.trajectory]
            pygame.draw.lines(surface, LIGHT_BLUE, False, points, 2)
        
        # 로봇 그리기 (이전과 동일)
        robot_screen_x = self.robot_x + offset_x
        robot_screen_y = self.robot_y + offset_y
        
        cos_a = math.cos(self.robot_angle)
        sin_a = math.sin(self.robot_angle)
        
        def rotate_point(x, y):
            rx = x * cos_a - y * sin_a
            ry = x * sin_a + y * cos_a
            return rx + robot_screen_x, ry + robot_screen_y
        
        body_points = [
            rotate_point(-25, -20), rotate_point(25, -20),
            rotate_point(25, 20), rotate_point(-25, 20)
        ]
        
        # 시뮬레이션 모드일 때 색상 변경
        color = YELLOW if self.simulation_mode else (GREEN if self.is_connected else BLUE)
        pygame.draw.polygon(surface, color, body_points)
        pygame.draw.polygon(surface, BLACK, body_points, 2)
        
        # (로봇의 나머지 부분 그리기 - 생략, 이전과 동일)
        # ... (바퀴, 캐스터, 핸들, 힘 화살표, 방향 표시)
        # 힘 화살표
        left_load = rotate_point(-45, -12)
        right_load = rotate_point(-45, 12)
        left_force = self.f_left / 15  # 음수/양수 모두 표시
        right_force = self.f_right / 15 # 음수/양수 모두 표시
        
        left_arrow_end = rotate_point(-45 - left_force, -12)
        pygame.draw.line(surface, RED, left_load, left_arrow_end, 3)
        
        right_arrow_end = rotate_point(-45 - right_force, 12)
        pygame.draw.line(surface, GREEN, right_load, right_arrow_end, 3)

        # 방향 표시
        arrow_points = [
            rotate_point(28, 0), rotate_point(35, -4), rotate_point(35, 4)
        ]
        pygame.draw.polygon(surface, YELLOW, arrow_points)
        

    # --- UI 그리기 (패널 내용 변경) ---
    # --- UI 그리기 (레이아웃 간격 조정) ---
    def draw_ui(self, surface):
        """UI 그리기 (겹침 해결 버전)"""
        # 1. 제목
        title = font_large.render("HSU-Smart Cart Simulation (RPM-Based)", True, BLACK)
        surface.blit(title, (20, 20))
        
        # 2. 캔버스 (왼쪽 메인 화면)
        # 캔버스 위치를 조금 위로 올려서 하단 공간 확보 (Y: 150 -> 100)
        canvas_y = 100
        self.draw_canvas(surface, 50, canvas_y)
        
        # 3. 왼쪽 하단 상태 표시 (캔버스 바로 아래)
        y_pos = canvas_y + 500 + 20 # 캔버스 높이(500) + 여백
        
        status_text = font_small.render(f"Arduino: {self.serial_status}", True, BLACK)
        surface.blit(status_text, (50, y_pos))
        
        mode_text = font_small.render(f"입력 모드: {self.input_mode.upper()}", True, BLACK)
        surface.blit(mode_text, (50, y_pos + 25))
        
        if self.simulation_mode:
            sim_status = "ON (시뮬레이션 모드)"
            sim_color = YELLOW
        else:
            sim_status = "OFF (실제 모터 구동)"
            sim_color = DARK_GRAY
            
        sim_text = font_small.render(f"시뮬레이션: {sim_status}", True, sim_color)
        surface.blit(sim_text, (50, y_pos + 50))
        
        # --- 우측 패널 영역 ---
        panel_x = 600
        panel_y = 100 # 캔버스와 높이 맞춤 (150 -> 100)
        
        # 4. 로드셀 입력 패널
        panel_h_load = 180
        self.draw_panel(surface, panel_x, panel_y, 350, panel_h_load, "로드셀 압력")
        
        y = panel_y + 40
        left_text = font_small.render(f"왼쪽 (F_left): {self.f_left:.0f}g", True, BLACK)
        surface.blit(left_text, (panel_x + 20, y))
        self.draw_bar(surface, panel_x + 20, y + 40, 310, self.f_left / 5000, RED)
        
        right_text = font_small.render(f"오른쪽 (F_right): {self.f_right:.0f}g", True, BLACK)
        surface.blit(right_text, (panel_x + 20, y + 60))
        self.draw_bar(surface, panel_x + 20, y + 100, 310, self.f_right / 5000, GREEN)

        # 5. 모터 모니터링 패널 (높이 대폭 확장)
        panel_y += panel_h_load + 20 # 이전 패널 높이 + 여백
        panel_h_motor = 430          # 내용이 많으므로 높이 확보 (400 -> 430)
        
        self.draw_panel(surface, panel_x, panel_y, 350, panel_h_motor, "모터 상세 정보")
        
        y = panel_y + 45
        line_gap = 32 # 줄 간격
        
        # 헤더 (Target vs Actual)
        header_font = font_small
        surface.blit(header_font.render("항목", True, DARK_GRAY), (panel_x + 20, y))
        surface.blit(header_font.render("Left", True, RED), (panel_x + 140, y))  # X좌표 간격 조정
        surface.blit(header_font.render("Right", True, GREEN), (panel_x + 250, y))
        
        y += 25
        pygame.draw.line(surface, GRAY, (panel_x+10, y), (panel_x+340, y), 1)
        y += 10

        # 1) 목표 속도 (Target RPM)
        surface.blit(font_small.render("Target RPM", True, BLACK), (panel_x + 20, y))
        surface.blit(font_small.render(f"{self.L_RPM_from_arduino:.0f}", True, BLACK), (panel_x + 140, y))
        surface.blit(font_small.render(f"{self.R_RPM_from_arduino:.0f}", True, BLACK), (panel_x + 250, y))
        y += line_gap

        # 2) 실제 속도 (Actual RPM) - 파란색 강조
        surface.blit(font_small.render("Actual RPM", True, BLUE), (panel_x + 20, y))
        surface.blit(font_small.render(f"{self.L_REAL_RPM:.0f}", True, BLUE), (panel_x + 140, y))
        surface.blit(font_small.render(f"{self.R_REAL_RPM:.0f}", True, BLUE), (panel_x + 250, y))
        y += line_gap
        
        # 3) 변환 속도 (m/s)
        surface.blit(font_small.render("Vel (m/s)", True, DARK_GRAY), (panel_x + 20, y))
        surface.blit(font_small.render(f"{self.Lvelocity:.2f}", True, DARK_GRAY), (panel_x + 140, y))
        surface.blit(font_small.render(f"{self.Rvelocity:.2f}", True, DARK_GRAY), (panel_x + 250, y))
        y += line_gap + 5 # 구분선 전 조금 더 띄움

        pygame.draw.line(surface, GRAY, (panel_x+10, y-10), (panel_x+340, y-10), 1)

        # 4) 실제 위치 (Position)
        surface.blit(font_small.render("Pos (cnt)", True, BLACK), (panel_x + 20, y))
        surface.blit(font_small.render(f"{self.L_POS}", True, BLACK), (panel_x + 140, y))
        surface.blit(font_small.render(f"{self.R_POS}", True, BLACK), (panel_x + 250, y))
        y += line_gap

        # 5) 실제 토크 (Torque)
        surface.blit(font_small.render("Trq (0.1%)", True, BLACK), (panel_x + 20, y))
        surface.blit(font_small.render(f"{self.L_TRQ}", True, BLACK), (panel_x + 140, y))
        surface.blit(font_small.render(f"{self.R_TRQ}", True, BLACK), (panel_x + 250, y))
        y += line_gap + 5

        # 6) 시뮬레이션 계산 값
        pygame.draw.line(surface, GRAY, (panel_x+10, y-10), (panel_x+340, y-10), 1)
        
        # draw_info_line 내부 좌표 수정이 필요할 수 있으므로 직접 그림
        surface.blit(font_small.render("Linear V:", True, PURPLE), (panel_x + 20, y))
        surface.blit(font_small.render(f"{self.velocity:.3f} m/s", True, PURPLE), (panel_x + 180, y))
        y += line_gap
        
        surface.blit(font_small.render("Angular W:", True, PURPLE), (panel_x + 20, y))
        surface.blit(font_small.render(f"{self.angular_vel:.3f} rad/s", True, PURPLE), (panel_x + 180, y))

        # 6. 조작 안내 (맨 오른쪽)
        # 위치를 X=1000으로 살짝 이동하여 중앙 패널과 겹치지 않게 함
        self.draw_controls(surface, 1000, 100)
    
    # --- UI 헬퍼 함수 ---
    def draw_panel(self, surface, x, y, w, h, title):
        """패널 그리기"""
        pygame.draw.rect(surface, WHITE, (x, y, w, h))
        pygame.draw.rect(surface, DARK_GRAY, (x, y, w, h), 2)
        
        title_text = font_medium.render(title, True, BLACK)
        surface.blit(title_text, (x + 10, y + 10))
    
    def draw_bar(self, surface, x, y, w, ratio, color):
        """진행 바 그리기 (음수 지원)"""
        pygame.draw.rect(surface, GRAY, (x, y, w, 20))
        bar_width = w * abs(ratio)
        if ratio >= 0:
            pygame.draw.rect(surface, color, (x, y, bar_width, 20))
        else:
            # 음수일 경우 (일단 왼쪽 정렬로 표시)
            pygame.draw.rect(surface, DARK_GRAY, (x, y, bar_width, 20))
        pygame.draw.rect(surface, BLACK, (x, y, w, 20), 1)
    
    def draw_info_line(self, surface, x, y, label, value, color=BLACK):
        """정보 라인 그리기"""
        label_text = font_small.render(label, True, DARK_GRAY)
        surface.blit(label_text, (x, y))
        
        value_text = font_small.render(value, True, color)
        surface.blit(value_text, (x + 180, y)) # 값 표시 위치 조정
    
    def draw_controls(self, surface, x, y):
        """조작 안내 그리기"""
        self.draw_panel(surface, x, y, 380, 400, "조작 방법")
        
        controls = [
            ("S", "시뮬레이션 모드 (ON/OFF)"),
            ("R", "리셋"),
            ("T", "경로 지우기"),
            ("C", "Arduino 연결"),
            ("D", "Arduino 연결 해제"),
            ("", ""),
            ("수동 모드:", "(Arduino 미연결 시)"),
            ("↑ / ↓", "왼쪽 로드셀 (+ / -)"),
            ("← / →", "오른쪽 로드셀 (+ / -)"),
            ("", ""),
            ("ESC", "종료")
        ]
        
        y_offset = y + 50
        for key, desc in controls:
            if key:
                key_text = font_small.render(key, True, BLUE)
                desc_text = font_small.render(desc, True, BLACK)
                surface.blit(key_text, (x + 20, y_offset))
                surface.blit(desc_text, (x + 120, y_offset))
            y_offset += 30

def main():
    sim = RobotSimulation()
    clock = pygame.time.Clock()
    running = True
    
    while running:
        dt = clock.tick(60) / 1000.0  # Delta time in seconds
        
        # 이벤트 처리
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                
                # 'S' 키로 시뮬레이션 모드 토글
                elif event.key == pygame.K_s:
                    sim.toggle_simulation_mode()
                
                # (제거됨) SPACE 키 로직
                # elif event.key == pygame.K_SPACE:
                #    sim.is_running = not sim.is_running
                
                elif event.key == pygame.K_r:
                    sim.reset()
                
                elif event.key == pygame.K_t:
                    sim.trajectory.clear()
                
                elif event.key == pygame.K_c:
                    if not sim.is_connected:
                        sim.connect_arduino()
                
                elif event.key == pygame.K_d:
                    if sim.is_connected:
                        sim.disconnect_arduino()
        
        # 수동 모드 키보드 입력 (후진 포함)
        if sim.input_mode == 'manual':
            keys = pygame.key.get_pressed()
            # C 코드 로직은 음수 값을 사용하므로 min/max 범위 변경
            if keys[pygame.K_UP]:
                sim.f_left = min(5000, sim.f_left + 20)
            if keys[pygame.K_DOWN]:
                sim.f_left = max(-5000, sim.f_left - 20)
            if keys[pygame.K_RIGHT]:
                sim.f_right = min(5000, sim.f_right + 20)
            if keys[pygame.K_LEFT]:
                sim.f_right = max(-5000, sim.f_right - 20)
        
        # 아두이노 데이터 읽기
        if sim.is_connected:
            sim.read_arduino_data()
        
        # 시뮬레이션 업데이트
        sim.update(dt)
        
        # 화면 그리기
        screen.fill(WHITE)
        sim.draw_ui(screen)
        pygame.display.flip()
    
    # 종료
    if sim.is_connected:
        sim.disconnect_arduino()
    
    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()
