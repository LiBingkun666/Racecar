import cv2
import numpy as np
import time
def buchang(p1,p2):
    if p1[0] > p2[0]:  # /斜线
        p1[0] += 50
        p2[0] += 50
        isleft = True
    else:# \斜线
        p1[0] -= 50
        p2[0] -= 50
        isleft = False
    return isleft
def getPoints(img,target):
    cor = np.argwhere(img == target)
    if len(cor) <= 0:
        return None,None
    xu1 = [cor[0][1] ,cor[0][0]]  # 第一个点
    xu2 = [cor[-1][1] ,cor[-1][0]]  # 最后一个点
    dist = np.linalg.norm(np.array(xu1) - np.array(xu2))
    if dist<30:
        return  None,None
    # img = cv2.circle(img,xu1,10,color=(0,0,255))
    # img = cv2.circle(img,xu2,10,color=(0,0,255))
    isLeft = buchang(xu1, xu2)
    return [xu1,xu2],isLeft

def drawLine(img,origin):
    xuxian,isLeft1 = getPoints(img,255) # 虚线
    huangxian,isLeft2 = getPoints(img,10) # 黄线
    shixian,isLeft3 = getPoints(img, 150)  # 实线
    xmin = 0
    xmax = 1000000
    if xuxian is not None:
        origin = cv2.line(origin, xuxian[0], xuxian[1], color=(0, 0, 255), thickness=10 )
        origin = cv2.putText(origin,"xuxian",xuxian[0],cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,255),2)
        if isLeft1:
            xmin = max(xmin,xuxian[0][0])
        else:
            xmax = min(xmax,xuxian[0][0])
    if huangxian is not None:
        origin = cv2.line(origin, huangxian[0], huangxian[1], color=(0, 0, 255), thickness=10)
        origin = cv2.putText(origin, "huangxian", huangxian[0], cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,255), 2)
        if isLeft2:
            xmin = max(xmin, huangxian[0][0])
        else:
            xmax = min(xmax, huangxian[0][0])
    if shixian is not None:
        origin = cv2.line(origin, shixian[0], shixian[1], color=(0, 0, 255), thickness=10)
        origin = cv2.putText(origin, "shixian", shixian[0], cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        if isLeft3:
            xmin = max(xmin, shixian[0][0])
        else:
            xmax = min(xmax, shixian[0][0])
    if xmax == 10000000:
        xmax = xmin
    return origin,(xmin,xmax)

# img = cv2.imread("11.jpg")
# origin = cv2.imread("output1.jpg")
# origin = drawLine(img,origin)
# cv2.imshow("x", origin)
# cv2.waitKey(0)