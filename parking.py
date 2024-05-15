#!/usr/bin/env python
#-- coding:utf-8 --
####################################################################
# 프로그램이름 : parking.py
# 코드작성팀명 : 오르다
####################################################################

#=============================================
# 함께 사용되는 각종 파이썬 패키지들의 import 선언부
#=============================================
import pygame
import numpy as np
import os
import math
import rospy
import heapq
from xycar_msgs.msg import xycar_motor

#=============================================
# 모터 토픽을 발행할 것임을 선언
#============================================= 
motor_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
xycar_msg = xycar_motor()

#=============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
#============================================= 
rx, ry = [300, 350, 400, 450, 1129], [300, 350, 400, 450, 69]

#=============================================
# 프로그램에서 사용할 상수 선언부
#=============================================
AR = (1142, 62) # AR 태그의 위치
P_ENTRY = (1036, 162) # 주차라인 진입 시점의 좌표
P_END = (1129, 69) # 주차라인 끝의 좌표

#=============================================
# 모터 토픽을 발행하는 함수
# 입력으로 받은 angle과 speed 값을
# 모터 토픽에 옮겨 담은 후에 토픽을 발행함.
#=============================================
def drive(angle, speed):
    xycar_msg.angle = int(angle)
    xycar_msg.speed = int(speed)
    motor_pub.publish(xycar_msg)

#=============================================
# 경로를 생성하는 함수
# 차량의 시작위치 sx, sy, 시작각도 syaw
# 최대가속도 max_acceleration, 단위시간 dt 를 전달받고
# 경로를 리스트를 생성하여 반환한다.
#=============================================

def heuristic(a, b):
    """ 유클리드 거리 휴리스틱 함수 """
    return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

def get_neighbors(state, max_acceleration, dt):
    """
    주어진 상태에서 가능한 다음 상태를 반환.
    - state: (x, y, yaw, speed)로 구성된 현재 상태.
    - max_acceleration: 최대 가속도.
    - dt: 시간 단위.
    """
    x, y, yaw, speed = state
    neighbors = []
    for acc in [-max_acceleration, 0, max_acceleration]:
        new_speed = speed + acc * dt
        new_speed = max(0, min(new_speed, 50))  # 속도는 0에서 50 사이로 제한
        new_x = x + new_speed * math.cos(yaw) * dt
        new_y = y + new_speed * math.sin(yaw) * dt
        for delta_yaw in [-50 * math.pi / 180, 0, 50 * math.pi / 180]:  # 각도 변경 범위 제한
            new_yaw = yaw + delta_yaw
            # new_yaw = (new_yaw + 2 * math.pi) % (2 * math.pi)  # 각도를 0~2π로 정규화
            neighbors.append((new_x, new_y, new_yaw, new_speed))
    return neighbors

def planning(sx, sy, syaw, max_acceleration, dt):
    """
    A* 알고리즘을 사용한 경로 계획 함수.
    - sx, sy: 시작 위치의 x, y 좌표.
    - syaw: 시작 시 차량의 각도 (방향).
    - max_acceleration: 최대 가속도.
    - dt: 시간 단위.
    """
    log_str = ""

    start = (sx, sy, syaw, 0)  # 초기 상태 (위치, 방향, 속도)
    goal_x, goal_y = P_END
    open_list = []
    heapq.heappush(open_list, (0, start))  # 우선순위 큐에 시작 상태 추가
    came_from = {}
    g_score = {start: 0}  # 시작 상태의 g_score 초기화
    f_score = {start: heuristic((sx, sy), (goal_x, goal_y))}  # 시작 상태의 f_score 초기화
    
    while open_list:
        current = heapq.heappop(open_list)[1]  # f_score가 가장 낮은 상태 선택
        x, y, yaw, speed = current
        strx, stry, stryaw, strspeed = map(str, current)
        log_str += strx+ "\t" +stry+ "\t" +stryaw+ "\t" +strspeed+'\n'
        if heuristic((x, y), (goal_x, goal_y)) < 1.0 and speed < 1.0:  # 목표 근처에 도달하고 속도가 0에 가까워지면 종료
            path = []
            while current in came_from:
                path.append(current)  # 경로를 역추적하여 추가
                current = came_from[current]
            path.append(start)
            path.reverse()
            
            # 경로를 리스트 형태로 변환
            rx = [int(state[0]) for state in path]
            ry = [int(state[1]) for state in path]

            # while문 안의 current를 저장해서 txt파일로 (저장 위치 : ~/.ros)
            tmp_str = ""
            for s in zip(rx, ry):
                tmp_str += str(s)
            log_str = tmp_str + log_str
            save_log(log_str)
            return rx, ry
        
        for neighbor in get_neighbors(current, max_acceleration, dt):
            tentative_g_score = g_score[current] + dt  # 시간 단위로 비용 계산
            
            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current  # 경로 추적을 위해 이전 상태 기록
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = g_score[neighbor] + heuristic((neighbor[0], neighbor[1]), (goal_x, goal_y))
                heapq.heappush(open_list, (f_score[neighbor], neighbor))  # 우선순위 큐에 새로운 상태 추가
    
    return None  # 경로를 찾지 못한 경우

#=============================================
# 생성된 경로를 따라가는 함수
# 파이게임 screen, 현재위치 x,y 현재각도, yaw
# 현재속도 velocity, 최대가속도 max_acceleration 단위시간 dt 를 전달받고
# 각도와 속도를 결정하여 주행한다.
#=============================================
def tracking(screen, x, y, yaw, velocity, max_acceleration, dt):
    global rx, ry
    print(x,y)
    # angle = 10 # -50 ~ 50
    speed = 0 # -50 ~ 50
    if x > 500:
        angle = 50
    else:
        angle = 10
    drive(angle, speed)


def save_log(string):
    print("save log 함수 작동")
    print(os.getcwd())
    with open('log.txt', 'w') as fx:
        fx.write(string)
    print("저장 프로세스 실행")