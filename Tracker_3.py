"""
    在Dtector对象中，查找结果为Detector.detectELL()返回值中的ellipse，
    对这个数组解压缩可以得到(x, y), (major, minor), angle三个值，反应椭圆的——位置，长短轴，角度
    通过椭圆左右两侧点数的多少区别进行方向的判断
"""


import cv2
import numpy as np
import skimage.morphology as sm
import skimage.util


class Tracker:
    def __init__(self,):#画面宽度， 高度， 目标半径
        #这里的参数除了found以外都只和摄像机与目标有关，和查找结果无关
        self.angle = 0   # 目标角度
        self.set_index()
        self.set_target()
        self.frame_threshold = None

        #记录两侧点数，用于判断旋转方向，以及后续函数显示
        self.pt_left = 0
        self.pt_right = 0
    
    def set_index(self, h = 480, w = 640, r = 100):#用于更新窗口的尺寸
        self.w = w
        self.h = h
        self.center = (w//2, h//2)
        self.minor = r
        self.major = r

    def set_target(self, 
                   lower = (60, 100, 100), upper = (85, 255, 255), 
                   lower_assist = (100, 80, 80), upper_assist = (125, 255, 255)):#重置目标参数，用于在完成一次动作后寻找下一个目标
        self.lower = lower  # 颜色筛选下限
        self.upper = upper  # 颜色筛选上限
        self.lower_assist = lower_assist  # 辅助颜色筛选上限
        self.upper_assist = upper_assist  # 辅助颜色筛选下限

# 椭圆检测部分
    def detectEll(self, frame):
        # 颜色筛选
        frame_HSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        frame_threshold = cv2.inRange(frame_HSV, self.lower, self.upper)
        # 根据光照条件和曝光参数 需要调整 最好在充足非直射白色光照的条件下检测
        
        # 中值滤波+面积筛选
        frame_threshold = cv2.medianBlur(frame_threshold, 5)
        frame_threshold = cv2.dilate(frame_threshold, None, iterations=2)#进行一次膨胀
        #frame_threshold = cv2.medianBlur(frame_threshold, 3)
        frame_threshold = cv2.erode(frame_threshold, None, iterations=3)#进行一次腐蚀
        #frame_threshold = cv2.erode(frame_threshold, None, iterations=3)#进行一次腐蚀
        #frame_threshold = cv2.dilate(frame_threshold, None, iterations=2)#进行一次膨胀

        frame_threshold = skimage.util.img_as_bool(frame_threshold)#将图片转化为二值图像
        frame_threshold = sm.remove_small_objects(frame_threshold, 130)#去除小面积
        frame_threshold = skimage.util.img_as_ubyte(frame_threshold)#转回uint8格式
        
        # 最小代数距离拟合（opencv有相应函数）
        ylist, xlist = frame_threshold.nonzero()# 得到非零点坐标
        #print("white pts num:" + str(len(xlist)))
        pts = (np.array([[xlist[i], ylist[i]] for i in range(len(xlist))], np.int32))
        
        if len(pts) < 100:
            found = False
            ellipse = self.center, (self.major, self.minor), self.angle
        else:
            # 拟合椭圆
            found = True
            ellipse = cv2.fitEllipse(pts)  # 拟合效果受到噪点影响非常严重
        frame_threshold = cv2.cvtColor(frame_threshold, cv2.COLOR_GRAY2BGR)#转回BGR格式,方便后续显示
        self.frame_threshold = frame_threshold#将检测后的图像存储在对象中

        #判断旋转方向
        self.pt_left = 0
        self.pt_right = 0
        target_x = ellipse[0][0]
        for i in range(len(xlist)):
            if xlist[i] < target_x:
                self.pt_left += 1
            elif xlist[i] > target_x:
                self.pt_right += 1
        if self.pt_left > self.pt_right:
            ConterWise  = False
        else:
            ConterWise  = True

        return found, ConterWise, ellipse
#
    def res_display(self, vision, res):
        #创建视图窗口
        window = np.full((self.h//2+80, self.w, 3), 50, np.uint8)
        
        frame_threshold = self.frame_threshold
        found, ConterWise, ellipse = res
        (x, y), (major, minor), angle = ellipse
        
        cv2.ellipse(vision, ((x, y), (major, minor), angle), (255, 255, 255), 2)
        cv2.circle(vision,(int(x),int(y)),3,(0,0,255),-1)
      
        vision_reshaped = cv2.resize(vision, (0, 0), fx=0.5, fy=0.5)
        window[0:self.h//2, 0:self.w//2, :] = vision_reshaped

        cv2.line(frame_threshold, (int(x), 0), (int(x), self.h), (255, 0, 0), 2)

        frame_threshold_reshaped = cv2.resize(frame_threshold, (0, 0), fx = 0.5, fy = 0.5)
        window[0:self.h//2, self.w//2:self.w, :] = frame_threshold_reshaped

        str1 = f"center:{('%.1f'%x, '%.1f'%y)}"
        str2 = f"major: {'%.0f'%major}, minor: {'%.0f'%minor}, angle: {'%.0f'%angle}, pt_left: {'%.0f'%self.pt_left}, pt_right: {'%.0f'%self.pt_right}"
        cv2.putText(window, str1, (10, self.h//2+25), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
        cv2.putText(window, str2, (10, self.h//2+50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
        if found:
            cv2.putText(window, "Target Found", (10, self.h//2+75), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        else:
            cv2.putText(window, "Target Not Found", (10, self.h//2+75), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        if ConterWise:
            cv2.putText(window, "CounterWise", (200, self.h//2+75), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        else:
            cv2.putText(window, "Not CounterWise", (200, self.h//2+75), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        #return window  #如果要对尺寸进行调整则输出一个返回值
        cv2.imshow("window", window)


if __name__ == '__main__':#用电脑镜头进行测试
    vision = cv2.VideoCapture(0)
    Tracker = Tracker()
    Tracker.set_target(lower=(40, 40, 80), 
                        upper=(80, 255, 255))
    Tracker.set_index(480, 640, 100)
    while True:
        ret, frame = vision.read()
        Tracker.set_index(frame.shape[0], frame.shape[1], 100)
        res = Tracker.detectEll(frame)
        Tracker.res_display(frame, res)
        #(x, y), (major, minor), angle = res[2]
        #print(f"center:{x, y}, major: {major}, minor: {minor}, angle: {angle}")
        if cv2.waitKey(20) & 0xFF == ord('q'):
            break
    cv2.destroyAllWindows()
    vision.release()
