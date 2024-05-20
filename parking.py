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
motor_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
xycar_msg = xycar_motor()

# 프로그램에서 사용할 변수, 저장공간 선언부
rx, ry = [], []

# 프로그램에서 사용할 상수 선언부
AR = (1142, 62)  # AR 태그의 위치
P_ENTRY = (1036, 162)  # 주차라인 진입 시점의 좌표
P_END = (1129, 69)  # 주차라인 끝의 좌표

# 차량 상태 변수
current_x, current_y, current_yaw = 0, 0, 0

def drive(angle, speed):
    xycar_msg.angle = int(angle)
    xycar_msg.speed = int(speed)
    rospy.loginfo(f"Driving command - Angle: {angle}, Speed: {speed}")
    motor_pub.publish(xycar_msg)

def bezier_curve(points, num=100):
    n = len(points)
    result = np.zeros((num, 2))
    for t in range(num):
        T = t / float(num - 1)
        temp = np.copy(points)
        for j in range(1, n):
            temp[:n - j, :] = (1 - T) * temp[:n - j, :] + T * temp[1:n - j + 1, :]
        result[t] = temp[0]
    return result[:, 0], result[:, 1]

def planning(sx, sy, syaw, max_acceleration, dt):
    global rx, ry
    rospy.loginfo("Start Planning")
    
    points = np.array([
        [sx, sy],
        [sx + 100, sy + 50],
        [P_ENTRY[0], P_ENTRY[1]],
        [P_END[0], P_END[1]]
    ])
    
    rx, ry = bezier_curve(points, num=50)
    return rx, ry

class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0

    def control(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        return self.Kp * error + self.Ki * self.integral + self.Kd * derivative

def pose_callback(data):
    global current_x, current_y
    current_x = data.pose.pose.position.x
    current_y = data.pose.pose.position.y
    rospy.loginfo(f"Pose updated - X: {current_x}, Y: {current_y}")

def imu_callback(data):
    global current_yaw
    current_yaw = data.orientation.z
    rospy.loginfo(f"Yaw updated - Yaw: {current_yaw}")

def tracking(screen, x, y, yaw, velocity, max_acceleration, dt):
    global rx, ry

    # PID 제어기 인스턴스 생성 및 초기 설정 값 조정
    pid = PIDController(Kp=1.5, Ki=0.1, Kd=0.05)
    
    # 가장 가까운 경로 점 계산
    target_index = np.argmin(np.sqrt((rx - x)**2 + (ry - y)**2))
    target_x = rx[target_index]
    target_y = ry[target_index]

    # 각도 오류 계산
    angle_error = math.atan2(target_y - y, target_x - x) - yaw
    
    # PID 제어 신호 계산
    angle = pid.control(angle_error, dt)
    
    # 각도 제한
    max_angle = 50  # 최대 조향각 (도)
    angle = max(min(angle, max_angle), -max_angle)
    
    # 속도 설정 (적절한 속도로 조정 필요)
    speed = 30  # 속도를 조정하여 경로 추적 성능 개선

    # 디버깅 출력을 추가
    rospy.loginfo(f"Target: ({target_x}, {target_y}), Current: ({x}, {y}), Angle Error: {angle_error}, Angle: {angle}")

    # 차량 제어
    drive(angle, speed)

def main():
    global current_x, current_y, current_yaw
    
    rospy.init_node('parking_node', anonymous=True)
    
    # 위치와 방향 데이터를 가져오는 ROS 토픽 구독
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, pose_callback)
    rospy.Subscriber('/imu', Imu, imu_callback)
    
    try:
        while not rospy.is_shutdown():
            # 갱신된 차량의 현재 상태를 사용
            x, y, yaw = current_x, current_y, current_yaw
            velocity = 0
            max_acceleration = 1.0
            dt = 0.1

            rx, ry = planning(x, y, yaw, max_acceleration, dt)
            tracking(None, x, y, yaw, velocity, max_acceleration, dt)

    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
