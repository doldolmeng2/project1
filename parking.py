#!/usr/bin/env python
#-- coding:utf-8 --
####################################################################
# 프로그램이름 : parking.py
# 코드작성팀명 : 오르다
####################################################################

#=============================================
# 함께 사용되는 각종 파이썬 패키지들의 import 선언부
#=============================================
import numpy as np
import math
import rospy
from xycar_msgs.msg import xycar_motor
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu

# 모터 토픽을 발행할 것임을 선언
motor_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1) #메세지를 발행할 토픽 이름, 발행할 메시지 타입, 발행할 메시지 큐의 크기
xycar_msg = xycar_motor() #메시지 객체 생성

# 프로그램에서 사용할 변수, 저장공간 선언부
rx, ry = [], []

# 프로그램에서 사용할 상수 선언부
AR = (1142, 62)  # AR 태그의 위치
P_ENTRY = (1036, 162)  # 주차라인 진입 시점의 좌표
P_END = (1129, 69)  # 주차라인 끝의 좌표

# 차량 상태 변수
current_x, current_y, current_yaw = 0, 0, 0

def drive(angle, speed):
    xycar_msg.angle = int(angle) #메시지에 각 정보를 담음
    xycar_msg.speed = int(speed) #메시지에 속도 정보를 담음
    rospy.loginfo(f"Driving command - Angle: {angle}, Speed: {speed}")
    motor_pub.publish(xycar_msg) #토픽에 메시지를 담아서 토픽을 발행함.

def bezier_curve(points, num=100):
    n = len(points)
    result = np.zeros((num, 2))
    for t in range(num):
        T = t / float(num - 1)
        temp = np.copy(points)
        for j in range(1, n):
            temp[:n - j, :] = (1 - T) * temp[:n - j, :] + T * temp[1:n - j + 1, :]
        result[t] = temp[0]
    return result[:, 0], result[:, 1] #전체 곡선을 구성하는 모든 점들의 x좌표, y좌표를 나눠서 리턴한다.

def planning(sx, sy, syaw, max_acceleration, dt):
    global rx, ry
    rospy.loginfo("Start Planning")
     
    points = np.array([ #베지어 곡선을 그리는데 사용되는 제어점(제어점이 곡선의 모양을 결정한다.)
        [sx, sy],
        [sx + 100, sy + 50],
        [P_ENTRY[0], P_ENTRY[1]],
        [P_END[0], P_END[1]]
    ])
    
    rx, ry = bezier_curve(points, num=50)
    return rx, ry

class PIDController: #비례-적분-미분 제어. 시스템의 현재 상태와 원하는 목표 상태 간의 차이, 즉 오차를 줄이기 위해 사용함.
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp #비례 제어 : 오차에 비례하여 제어 액션을 취함. Kp*error
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0

    def control(self, error, dt):
        self.integral += error * dt #적분 제어 : 시간에 대해 누적된 오차를 줄이기 위해 사용
        derivative = (error - self.prev_error) / dt #미분 제어 : 오차가 얼마나 빠르게 변하는지에 대해 반응
        self.prev_error = error
        return self.Kp * error + self.Ki * self.integral + self.Kd * derivative

def pose_callback(data):  #현재 자동차의 위치를 업데이트한다.
    global current_x, current_y
    current_x = data.pose.pose.position.x
    current_y = data.pose.pose.position.y
    rospy.loginfo(f"Pose updated - X: {current_x}, Y: {current_y}")

def imu_callback(data): #현재 자동차의 각을 업데이트한다.
    global current_yaw
    current_yaw = data.orientation.z
    rospy.loginfo(f"Yaw updated - Yaw: {current_yaw}")

def tracking(screen, x, y, yaw, velocity, max_acceleration, dt):
    global rx, ry

    # PID 제어기 인스턴스 생성 및 초기 설정 값 조정
    #kp,ki,kd 매개변수를 조정해서 경로 추적 성능을 개선할 수 있다.
    pid = PIDController(Kp=1.5, Ki=0.1, Kd=0.05)
    
    # 가장 가까운 경로 점 계산
    target_index = np.argmin(np.sqrt((rx - x)**2 + (ry - y)**2)) #가장 값이 작은 원소의 인덱스를 리턴함.
    target_x = rx[target_index] #우리가 목표로하는 베지어 곡선 위의 점의 x좌표
    target_y = ry[target_index] #우리가 목표로하는 베지어 곡선 위의 점의 y좌표

    # 각도 오류 계산
    angle_error = math.atan2(target_y - y, target_x - x) - yaw #두 변수를 받아서 그들의 비율에 대한 아크탄젠트 값을 반환한다. 단위가 '도'인 것으로 보인다.
    #두 점(목표x,목표y)와 (현재x,현재y) 사이의 각도를 계산
    #원점에서 출발해서 목표점으로 향하는 벡터와, 원점에서 출발해서 현재점으로 향하는 벡터 사이의 각을 구한다.


    # PID 제어 신호 계산
    angle = pid.control(angle_error, dt) #dt시간에 대한 PID제어를 적용한 각을 받는다.
    rospy.loginfo(f"pid controlled angle : {angle}")
    #로그를 살펴봤을 때, pid.control을 거친 각이 -240언저리가 찍힘. 무언가 잘못되었음. PID 파라미터를 조정하거나 코드상의 오류를 수정해야 할 듯

    if angle>180:
        angle=180-angle
    elif angle<-180:
        angle=-180-angle

    # 각도 제한
    max_angle = 50  # 최대 조향각 (도)
    angle = max(min(angle, max_angle), -max_angle) #50~-50도로 조향각을 제한한다.
    
    # 속도 설정 (적절한 속도로 조정 필요)
    speed = 30  # 속도를 조정하여 경로 추적 성능 개선

    # 디버깅 출력을 추가
    rospy.loginfo(f"Target: ({target_x}, {target_y}), Current: ({x}, {y}), Angle Error: {angle_error}, Angle: {angle}")

    # 차량 제어
    drive(angle, speed)

def main():
    global current_x, current_y, current_yaw
    
    rospy.init_node('parking_node', anonymous=True) #ROS환경에서 파이썬 노드 생성.
    
    # 위치와 방향 데이터를 가져오는 ROS 토픽 구독
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, pose_callback) #구독할 토픽 이름, 구독할 메시지 타입, 메시지를 받았을 때 실행한 콜백 함수
    rospy.Subscriber('/imu', Imu, imu_callback)
    
    try:
        while not rospy.is_shutdown():
            # 갱신된 차량의 현재 상태를 사용
            x, y, yaw = current_x, current_y, current_yaw
            velocity = 0
            max_acceleration = 1.0
            dt = 0.1

            rx, ry = planning(x, y, yaw, max_acceleration, dt) #곡선 정보를 받음.(rx는 모든 점들의 x좌표를 리스트로 가지고 있고, ry는 y좌표)
            #rx,ry는 tracking함수 내에서 전역 변수로 선언되어서 사용된다.
            tracking(None, x, y, yaw, velocity, max_acceleration, dt)

    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
