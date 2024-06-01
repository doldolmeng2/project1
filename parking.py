#!/usr/bin/env python
#-- coding:utf-8 --
####################################################################
# 프로그램이름 : parking.py
# 코드작성팀명 : 오르다
####################################################################

#=============================================
# 함께 사용되는 각종 파이썬 패키지들의 import 선언부

import pygame
import numpy as np
import os
import math
import rospy
import heapq
import time
from xycar_msgs.msg import xycar_motor


# 모터 토픽을 발행할 것임을 선언 
motor_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
xycar_msg = xycar_motor()


# 프로그램에서 사용할 변수, 저장공간 선언부 
rx, ry, ryaw  = [], [], []

# 프로그램에서 사용할 상수 선언부
AR = (1142, 62) # AR 태그의 위치
P_ENTRY = (1036, 162) # 주차라인 진입 시점의 좌표
P_END = (1129, 69) # 주차라인 끝의 좌표 (진입 시점과의 차이 93, -93)
DECIMAL_POINT = 5
last_target_index = 1

# 모터 토픽을 발행하는 함수
# 입력으로 받은 angle과 speed 값을
# 모터 토픽에 옮겨 담은 후에 토픽을 발행함.
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

def calculate_third_side(a, b, theta):  # 코사인 2법칙으로 두 변과 끼인각으로 나머지 한 변을 구하는 메서드
    c = math.sqrt(a**2 + b**2 - 2*a*b*math.cos(theta))
    return c

def heuristic(x, y): # 유클리드 거리 기반 휴리스틱 함수
    end_x, end_y = P_ENTRY
    score = math.sqrt((end_x - x) ** 2 + (end_y - y) ** 2)
    return score

def cal_distance(sx, sy, rx, ry): # 단순히 두 좌표의 거리를 계산
    return math.sqrt((rx - sx)**2 + (ry - sy)**2)

def neighbors(x, y, yaw, speed, acc, dt, degree): # 어떤 좌표에서 다음 지점의 좌표를 구하는 메서드
    neighbors = []
    for delta_yaw in degree: # 좌,직,우 각도를 하나씩 만듦
        new_yaw = yaw + delta_yaw # 새로운 각도 값 생성
        new_speed = speed + acc * dt # 새로운 속도 값 생성
        # 속도 -50 ~ 50 사이로 제한
        if abs(new_speed) > 50:
            if new_speed < 0:
                new_speed = -50
            else:
                new_speed = 50
        new_x = x + -(new_speed * math.sin(math.radians(new_yaw)) * dt * 3) #새로운 x위치
        new_y = y + new_speed * math.cos(math.radians(new_yaw)) * dt * 3    #새로운 y위치
    
        # x,y 값을 제한해서 밖으로 나가면 찾지 않고 다음 노드 탐색
        if new_x < 0 or new_x > 1300:
            continue
        elif new_y < 0 or new_y > 1300:
            continue
        # 이웃 노드에 추가추가
        neighbors.append(tuple([round(i,5) for i in (new_x, new_y, new_yaw, new_speed)]))
    return neighbors





def planning(sx, sy, syaw, max_acceleration, dt):
    global rx, ry, ryaw
    cnt = 0 # while문 scan횟수
    
    # A* 알고리즘 구현
    start = (sx, sy, syaw, 0)  # 초기 상태
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(sx, sy)}
    #후진을 위한 변수 초기화
    reverse_x = []
    reverse_y = []
    reverse_p = 3
    # log를 저장하기 위한 변수 초기화
    re_str = ""
    print(sx,sy)
    while open_set:

        # 만약 cnt가 1499까지 전진 방향으로 방법을 찾지 못하면 후진으로 노드 탐색
        if cnt == 1501:
            # print("reverse mode", "cnt :", cnt )
            # print(start)
            reverse_cnt = 0
            current = start
            reverse_x = []
            reverse_y = []
            while reverse_cnt < 50 * reverse_p: # 한 번 실행될 떄마다 reverse_p 를 늘려서 더 많은 후진까지 탐색함
                min_h = float("inf")
                for neighbor in neighbors(*current, -max_acceleration, dt, [-0.2, 0, 0.2]): # 후진 방향으로 노드 탐색
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
        
        if heuristic(current[0], current[1]) < 1.0: # 만약 현재 탐색 위치와 도착 위치 차이가 1 미만이면
            rx = []
            ry = []
            ryaw = []
            # 경로 역추적으로 저장
            while current in came_from:
                rx.append(current[0])
                ry.append(current[1])
                ryaw.append(current[2])
                current = came_from[current]
            rx.append(start[0])
            ry.append(start[1])
            ryaw.append(start[2])
            # 역추적 됐으므로 뒤집는다
            rx.reverse()
            ry.reverse()
            ryaw.reverse()
            # print(rx[-1], ry[-1], "  cnt =",cnt, "dt:", dt)
            # print("len(rx): ", len(rx), "   len(ry): ",len(ry))
            # print("len(reverse_x)", len(reverse_x), "  len(reverse_y)", len(reverse_y))

            # P_ENTRY에서 끝가지 노드를 만듦
            arrival_x = list(map(lambda x: round(x, 1), np.arange(P_ENTRY[0],P_END[0],2.4)))
            arrival_y = list(map(lambda x: round(x, 1), np.arange(P_ENTRY[1],P_END[1],-2.4)))
            rx.extend(arrival_x) 
            ry.extend(arrival_y)
            rx = reverse_x + rx
            ry = reverse_y + ry
            for point in tuple(zip(rx, ry)):
                re_str += str(point) + "\t" + str(heuristic(point[0], point[1])) + "\n" 
            # save_log(re_str)
            # print("두 번째 까지 거리", cal_distance(rx[1], ry[1], rx[0], ry[0]))
            # print("세 번째 까지 거리", cal_distance(rx[2], ry[2], rx[1], ry[1]))
            # print("네 번째 까지 거리", cal_distance(rx[3], ry[3], rx[2], ry[2]))
            # print("다섯 번째 까지 거리", cal_distance(rx[4], ry[4], rx[3], ry[3]))
            # print("여섯 번째 까지 거리", cal_distance(rx[5], ry[5], rx[4], ry[4]))
            # print("일곱 번째 까지 거리", cal_distance(rx[6], ry[6], rx[5], ry[5]))
            return rx, ry
                
        # A* 알고리즘의 핵심
        # 이웃노드를 만들어서 현재까지의 값 + 도착 위치 까지의 값으로 비교한다.
        for neighbor in neighbors(*current, max_acceleration, dt, [-0.61, 0, 0.61]):
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


class PIDController:
    """비례-적분-미분(PID) 제어를 위한 클래스."""
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.last_error = 0

    def compute(self, error, dt):
        """PID 제어 계산을 수행하고 조정된 제어 값을 반환."""
        self.integral += error * dt
        derivative = (error - self.last_error) / dt if dt > 0 else 0
        self.last_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

def tracking(screen, x, y, yaw, velocity, max_acceleration, dt):
    global rx, ry, PID_controller, target_index, last_target_index

    if not rx or not ry:
        rospy.loginfo("경로 정보가 없습니다.")
        return
    ## reverse mode 설정
    reverse_mode = False
    # 가장 가까운 경로 점 찾기
    distances = [math.sqrt((rx[i] - x)**2 + (ry[i] - y)**2) for i in range(len(rx))]
    nearest_index = distances.index(min(distances))

    # 경로를 따라가기 위해 다음 목표점 설정
    ## 좀 더 먼 곳을 목표지점으로 하기 위해서 '+ 25' 로 설정했습니다
    ## 따라서 주차시에도 일찍 회전하게돼서 알맞게 들어갑니다
    new_target_index = nearest_index + 25 if nearest_index + 25 < len(rx) else len(rx)-1
    
    # 후진 -> 전진 과정에서의 target_index 오류 해결을 위한 index 제한
    if new_target_index + 100 < last_target_index:
        target_index = new_target_index
    elif new_target_index < last_target_index:
        target_index = last_target_index
    elif new_target_index > last_target_index:
        target_index = new_target_index
    else:
        target_index = last_target_index
    last_target_index = target_index
    
    target_x, target_y = rx[target_index], ry[target_index]

    # 목표점까지의 각도 계산
    path_angle = math.degrees(math.atan2(y - target_y, target_x - x))
    ## path_angle - yaw 에서 변경 
    angle_error = yaw - path_angle

    #후진 해야 할 각도에서는 180를 +-
    if angle_error > 180:
        angle_error -= 180          
        angle_error = -angle_error  # reverse mode 이면 조향 방향을 반대로
        reverse_mode = True         # reverse mode on
    elif angle_error < -180:
        angle_error += 180
        angle_error = -angle_error
        reverse_mode = True
    angle_error = (angle_error + 360) % 360
    if angle_error > 180:
        angle_error -= 360  # 최소 회전 각도로 조정


    # PID 컨트롤러를 사용하여 조향각 계산
    ## Computed PID 여러번 하면 계속 적분값이 누적돼서 compute_PID로 다 통일 했습니다.
    compute_PID = PID_controller.compute(angle_error, dt)
    
    steering_angle = min(max(compute_PID, -50), 50)  # 최대 조향각을 50도로 제한
    print("target_index : ", target_index)
    print("rx ry : ", rx[0], ry[0])
    rospy.loginfo(f"{x} // {y} // {target_x} // {target_y} // {reverse_mode} ")
    # 속도 설정
    max_speed = 50  # 최대 속도
    min_speed = -50   # 최소 속도 설정
    
    ## 속도를 velocity, max_acceleration, dt 값을 활용해서 계산하도록 바꾸었습니다.
    ## END포인트와 65미만으로 거리차이가 나면 감속
    if cal_distance(x, y, P_END[0], P_END[1]) < 65:
        speed = max(velocity - max_acceleration * dt, 0)
    else:
        if reverse_mode:
            speed = max(min_speed, velocity - max_acceleration * dt)
        else:
            speed = min(max_speed, velocity + max_acceleration * dt)
    

    drive(steering_angle, speed)

# PID 컨트롤러 초기화 (파라미터는 실험을 통해 조정 필요)
PID_controller = PIDController(kp=5, ki=0, kd=0)