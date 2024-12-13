import cv2
import numpy as np
import time
import djitellopy as tello
import math

class Control():
    def __init__(self, x_target, y_target, r_target, speed_limit, 
                 kp = 0.5, ki = 0.01, kd = 0.001, dr = 0.9):
        
        #目标参数
        self.update_target(x_target, y_target, r_target, speed_limit)

        #当前各个通道的运行速度
        self.v_x = 0
        self.v_y = 0
        self.v_angel = 0
        self.v_distance = 0

        #控制参数
        self.update_PID(kp, ki, kd, dr)

        #PID参数
        self.intergral_vx = 0
        self.intergral_vy = 0
        self.intergral_vangel = 0
        self.intergral_vdistance = 0

        self.derivative_vx = 0
        self.derivative_vy = 0
        self.derivative_vangel = 0
        self.derivative_vdistance = 0


    
    def update_target(self, x_target, y_target, r_target, speed_limit):
        self.x_target = x_target
        self.y_target = y_target
        self.r_target = r_target
        self.speed_limit = speed_limit

    def update_PID(self, kp, ki, kd, dr):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.decrease_rate = dr

    def calculate_speed(self, ellipse, conterwise):
        (x_current, y_current), (minor_current, major_current), tmp = ellipse
        dx = self.x_target - x_current
        dy = self.y_target - y_current
        dangel = (major_current - minor_current)/minor_current*self.r_target#注意，这是一个float类型
        ddistance = self.r_target - major_current//2

        dx_corrected = dx * 0.15
        dy_corrected = dy * 0.15
        dangel_corrected = abs(dangel)*0.10
        ddistance_corrected = ddistance * 0.30

        #计算速度
        vx = self.kp * dx_corrected + self.ki * self.intergral_vx - self.kd * self.derivative_vx
        vy = self.kp * dy_corrected + self.ki * self.intergral_vy - self.kd * self.derivative_vy
        vangel = self.kp * dangel_corrected + self.ki * self.intergral_vangel - self.kd * self.derivative_vangel
        vdistance = self.kp * ddistance_corrected + self.ki * self.intergral_vdistance - self.kd * self.derivative_vdistance

        #修正正负情况
        vx = -vx

        #更新积分参数
        self.intergral_vx += vx
        self.intergral_vx *= self.decrease_rate
        self.intergral_vy += vy
        self.intergral_vy *= self.decrease_rate
        self.intergral_vangel += vangel
        self.intergral_vangel *= self.decrease_rate
        self.intergral_vdistance += vdistance
        self.intergral_vdistance *= self.decrease_rate

        #更新微分参数
        self.derivative_vx = vx - self.v_x + self.derivative_vx*self.decrease_rate
        self.derivative_vy = vy - self.v_y + self.derivative_vy*self.decrease_rate
        self.derivative_vangel = vangel - self.v_angel + self.derivative_vangel*self.decrease_rate
        self.derivative_vdistance = vdistance - self.v_distance + self.derivative_vdistance*self.decrease_rate

        #设置上下限度
        if vx> self.speed_limit:
            vx = self.speed_limit
        elif vx < -self.speed_limit:
            vx = -self.speed_limit
        
        if vy> self.speed_limit:
            vy = self.speed_limit
        elif vy < -self.speed_limit:
            vy = -self.speed_limit
        
        if vdistance> self.speed_limit:
            vdistance = self.speed_limit
        elif vdistance < -self.speed_limit:
            vdistance = -self.speed_limit
        
        if vangel> self.speed_limit//3:
            vangel = self.speed_limit//3
        elif vangel < -self.speed_limit//3:
            vangel = -self.speed_limit//3

        vangel = abs(vangel)

        
        if not conterwise:
            vangel = - vangel
        else:
            vangel = vangel
        #计算角速度(用于处理弧度制与角度制的差异)
        vangel = vangel * 10


        #更新速度
        self.v_x = vx
        self.v_y = vy
        self.v_angel = vangel
        self.v_distance = vdistance

        #print("Vangle: ", self.v_angel)
        
    def move_tello(self, tello):
        acceptable_err = 6  
        x_ac = False
        y_ac = False
        d_ac = False
        a_ac = False
        #tello.send_rc_control(int(self.v_distance),  int(self.v_x), int(self.v_y), int(self.v_angel))
        if(self.v_x>acceptable_err):
            tello.move_right(20)
        elif(self.v_x<-acceptable_err):
            tello.move_left(20)
        else:   
            x_ac = True
        
        if(self.v_y>acceptable_err):
            tello.move_up(20)
        elif(self.v_y<-acceptable_err):
            tello.move_down(20)
        else:
            y_ac = True
        
        if(self.v_angel>acceptable_err//2):
            tello.rotate_clockwise(10)
        elif(self.v_angel<-acceptable_err//2):
            tello.rotate_counter_clockwise(10)
        else:
            a_ac = True

        if(self.v_distance>acceptable_err):
            tello.move_forward(20)
        elif(self.v_distance<-acceptable_err):
            tello.move_back(20)
        else:
            d_ac = True

        if(x_ac and y_ac and d_ac and a_ac):
            return True
        else:
            return False
        
        #time.sleep(0.5)

    def display_velocity(self):
        window = np.zeros((400, 500, 3), np.uint8)
        cv2.putText(window, "Vx: " + str(self.v_x), (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        cv2.putText(window, "Vy: " + str(self.v_y), (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        cv2.putText(window, "Vangle: " + str(self.v_angel), (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        cv2.putText(window, "Vdistance: " + str(self.v_distance), (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        cv2.imshow("Velocity", window)