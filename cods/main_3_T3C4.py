import cv2
import numpy as np
import time
import threading
import djitellopy as tello

import Tracker_3 as Tracker
import Control_4 as Control

def display_velocity(velocity, soc):
    v_x , v_y, v_angel, v_distance = velocity
    window = np.zeros((120, 500, 3), np.uint8)
    cv2.putText(window, "Vx: " + str(v_x), (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    cv2.putText(window, "Vy: " + str(v_y), (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    cv2.putText(window, "Vangle: " + str(v_angel), (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    cv2.putText(window, "Vdistance: " + str(v_distance), (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    cv2.putText(window, "Soc: " + str(soc), (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    cv2.imshow("Velocity", window)


#需要共享的变量
done = False
found = False
conterwise = True
AC = False
FINISHED = False
ellipse = ((320, 220), (50, 50), 0)
target = (320, 220, 100)
velocity = (0, 0, 0, 0)
soc = 100

def Detect():#处理视频
    global done, target, found, ellipse, conterwise, velocity, soc, AC, FINISHED

    TELLO.streamon()
    #vision = cv2.VideoCapture(0)
    while True:
        frame = TELLO.get_frame_read().frame
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        #_, frame = vision.read()
        #print(frame.shape)
        TRACKER.set_index(frame.shape[0], frame.shape[1], 300)
        res = TRACKER.detectEll(frame)
        TRACKER.res_display(frame, res)

        display_velocity(velocity, soc)
        
        #更新共享数据
        target = (TRACKER.w//2, TRACKER.h//2, TRACKER.major/2)
        found = res[0]
        conterwise = res[1]
        ellipse = res[2]
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            done = True
            TELLO.streamoff()
            TELLO.land()
            cv2.destroyAllWindows()
            break
        
        if AC:
            done = True
        
        if FINISHED :
            TELLO.streamoff()
            TELLO.land()
            cv2.destroyAllWindows()
            break

def Move():#进行运动
    global done, target, found, ellipse, conterwise, velocity, soc, AC, FINISHED
    velocity = (0, 0, 0, 0)#防止起飞之后乱飞
    TELLO.takeoff()
    
    while not done:
        CONTROL.update_target(target[0], target[1], target[2], 20)
        soc = TELLO.get_battery()
        
        if AC:
            print("FINISHED!!!")    
            TELLO.move_down(20)
            TELLO.move_forward(250)
            TELLO.land()
            TELLO.end()
            FINISHED = True
            break
        
        if found:
            CONTROL.calculate_speed(ellipse, conterwise)
            velocity = (CONTROL.v_x, CONTROL.v_y, CONTROL.v_angel, CONTROL.v_distance)
            AC = CONTROL.move_tello(TELLO)
        else:
            velocity = (0, 0, 0, 0)
            time.sleep(0.2) #防止CPU占用过高

def Move_testing():
    global done, target, found, ellipse, conterwise, velocity, soc, FINISHED
    velocity = (0, 0, 0, 0)#防止起飞之后乱飞

    while not done and not AC:
        CONTROL.update_target(target[0], target[1], target[2], 100)

        CONTROL.calculate_speed(ellipse)
        velocity = (CONTROL.v_x, CONTROL.v_y, CONTROL.v_angel, CONTROL.v_distance)
        CONTROL.display_velocity()
        time.sleep(0.5)


if __name__ == '__main__':
    TELLO = tello.Tello()
    TELLO.connect()

    TRACKER = Tracker.Tracker()
    TRACKER.set_target(lower=(50, 60, 40), 
                       upper=(80, 255, 255))
    TRACKER.set_index(480, 640, 300)

    CONTROL = Control.Control(TRACKER.w//2, TRACKER.h//2, TRACKER.major/2, 300)
    CONTROL.update_PID(0.5, 0.1, 0.01, 0.2)#kp, ki, kd, dr

    thread1 = threading.Thread(target=Detect)
    thread2 = threading.Thread(target=Move)
    #thread2 = threading.Thread(target=Move_testing)

    thread1.start()
    thread2.start()

    thread2.join()
    thread1.join()
    
    TELLO.land()
    TELLO.end()