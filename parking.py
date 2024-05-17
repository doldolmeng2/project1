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
P_END = (1129, 69) # 주차라인 끝의 좌표 (진입 시점과의 차이 93, -93)
P_S = (946, 252)
DECIMAL_POINT = 5

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
def calculate_third_side(a, b, theta):
    """
    주어진 두 변(a, b)과 그 사이의 각(theta)을 이용하여 세 번째 변(c)을 계산합니다.
    theta는 라디안 단위여야 합니다.
    """
    c = math.sqrt(a**2 + b**2 - 2*a*b*math.cos(theta))
    return c

def heuristic(x, y):
    """유클리드 거리 기반 히유리스틱 함수."""
    end_x, end_y = P_S
    score = math.sqrt((end_x - x) ** 2 + (end_y - y) ** 2)
    return score

def cal_distance(sx, sy, rx, ry):
    return math.sqrt((rx - sx)**2 + (ry - sy)**2)

def neighbors(x, y, yaw, speed, acc, dt, degree):
    neighbors = []
    for delta_yaw in degree:
        new_yaw = yaw + delta_yaw
        new_speed = speed + acc * dt
        # 속도 50 이하로 제한
        if abs(new_speed) > 50:
            if new_speed < 0:
                new_speed = -50
            else:
                new_speed = 50
        new_x = x + -(new_speed * math.sin(math.radians(new_yaw)) * dt * 4)
        new_y = y + new_speed * math.cos(math.radians(new_yaw)) * dt * 4
        neighbors.append(tuple([round(i,5) for i in (new_x, new_y, new_yaw, new_speed)]))
    return neighbors

def cal_degree(now_x, now_y, target_x, target_y):
    return -(math.degrees(math.atan2(target_y - now_y, target_x - now_x)))



def planning(sx, sy, syaw, max_acceleration, dt):
    global rx, ry
    """자율주행 경로 생성 함수."""

    cnt = 0
    start = (sx, sy, syaw, 0)  # 초기 상태
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(sx, sy)}
    reverse_x = []
    reverse_y = []

    reverse_p = 3
    
    while open_set:

        if cnt == 1500:
            print("reverse mode", "cnt :", cnt )
            print(start)
            reverse_cnt = 0
            current = start
            reverse_x = []
            reverse_y = []
            while reverse_cnt < 50 * reverse_p:
                min_h = float("inf")
                for neighbor in neighbors(*current, -max_acceleration, dt, [-0.2, 0, 0.2]):
                    if min_h > heuristic(neighbor[0], neighbor[1]):
                        min_h = heuristic(neighbor[0], neighbor[1])
                        current = neighbor
                last_yaw = neighbor[2]
                reverse_x.append(neighbor[0])
                reverse_y.append(neighbor[1])
                reverse_cnt += 1
            _start = (reverse_x[-1], reverse_y[-1], last_yaw, 0)  # 초기 상태
            open_set = []
            heapq.heappush(open_set, (0, _start))
            came_from = {}
            g_score = {_start: 0}
            f_score = {_start: heuristic(_start[0], _start[1])}
            reverse_p += 1 
            cnt = 0
        _, current = heapq.heappop(open_set)
        
        if heuristic(current[0], current[1]) < 1.0:
            rx = []
            ry = []
            while current in came_from:
                rx.append(current[0])
                ry.append(current[1])
                current = came_from[current]
            rx.append(start[0])
            ry.append(start[1])
            rx.reverse()
            ry.reverse()

            # print(rx[-1], ry[-1], "  cnt =",cnt, "dt:", dt)
            print("len(rx): ", len(rx), "   len(ry): ",len(ry))
            # print("len(reverse_x)", len(reverse_x), "  len(reverse_y)", len(reverse_y))

            arrival_x = list(map(lambda x: round(x, 1), np.arange(P_S[0],P_END[0],2.4)))
            arrival_y = list(map(lambda x: round(x, 1), np.arange(P_S[1],P_END[1],-2.4)))
            rx.extend(arrival_x) 
            ry.extend(arrival_y)
            rx = reverse_x + rx
            ry = reverse_y + ry

            print("두 번째 까지 거리", cal_distance(rx[1], ry[1], rx[0], ry[0]))
            print("세 번째 까지 거리", cal_distance(rx[2], ry[2], rx[1], ry[1]))
            print("네 번째 까지 거리", cal_distance(rx[3], ry[3], rx[2], ry[2]))
            print("다섯 번째 까지 거리", cal_distance(rx[4], ry[4], rx[3], ry[3]))
            print("여섯 번째 까지 거리", cal_distance(rx[5], ry[5], rx[4], ry[4]))
            print("일곱 번째 까지 거리", cal_distance(rx[6], ry[6], rx[5], ry[5]))

            return rx, ry
                

        for neighbor in neighbors(*current, max_acceleration, dt, [-0.8, 0, 0.8]):
            tentative_g_score = g_score[current] + dt
            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor[0], neighbor[1])
                heapq.heappush(open_set, (f_score[neighbor], neighbor))
        cnt += 1
    



#=============================================
# 생성된 경로를 따라가는 함수
# 파이게임 screen, 현재위치 x,y 현재각도, yaw
# 현재속도 velocity, 최대가속도 max_acceleration 단위시간 dt 를 전달받고
# 각도와 속도를 결정하여 주행한다.
#=============================================

track_cnt = 0
def tracking(screen, x, y, yaw, velocity, max_acceleration, dt):
    global rx, ry
    global track_cnt
    # angle = 10 # -50 ~ 50

    # process
    # rx, ry 값을 탐색하며 목표위치 결정
    # 현재 위치와 목표위치까지 거리가 일정 값 미만이 되면 다음 목표위치 탐색 (다음 rx,ry값 탐색)
    # 현재 yaw와 목표위치까지 각도를 계산해서 각도 적용.
    # abs(목표 각도 - 현재각도) >0 이면 후진(speed가 음수)

    # 첫 번째에서 두번째 까지의 거리는 약 1.02 
    # 두 번째 까지는 약 2.04
    # 세 번째 까지는 약 3.07
    # 네 번째 이상부터는 까지는 약 3.2

    if rx and ry:
        if cal_distance(rx[0], ry[0], x, y) < 3.5: # 목표 까지의 거리가 3보다 작으면 rx,ry 변경
            rx = rx[1:]
            ry = ry[1:]
        
    

    # test
    # if track_cnt == 0:
    #     print(" tracking 시작값 x:", x, "   y:",y, "   yaw :", yaw)
    # elif track_cnt < 10:
    #     print(f'track_cnt :{track_cnt} 에서 velocity:{velocity}') # cnt 2부터 속도 50

    if track_cnt == 100 or track_cnt == 500:
        print("cnt 100, 500일 때 속도 :", velocity)
    track_cnt += 1
    angle = 50
    speed = 100

    drive(angle, speed)


def save_log(string):
    print("save log 함수 작동")
    print(os.getcwd())
    with open('log.txt', 'w') as fx:
        fx.write(string)
    print("저장 프로세스 실행")

#