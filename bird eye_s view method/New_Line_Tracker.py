import cv2
import numpy as np
from matplotlib import pyplot as plt
from polydrawer import Polydrawer
from polyfitter import Polyfitter
from thresholder import Thresholder
from undistorter import Undistorter
from warper import Warper
import time
i = 0
undistorter = Undistorter()
thresholder = Thresholder()
warper = Warper()
polyfitter = Polyfitter()
polydrawer = Polydrawer()


video1 = cv2.VideoCapture("project_video.mp4")

while True:
    
    ret, video = video1.read()
    if not ret:
        video = cv2.VideoCapture("project_video.mp4")
        continue
    i += 1
    #undistorted = undistorter.undistort(video)
    img = cv2.cvtColor(video, cv2.COLOR_RGB2HSV)

    yellow_min = np.array([70, 64, 123], np.uint8)
    yellow_max = np.array([100, 255, 255], np.uint8)
    yellow_mask = cv2.inRange(img, yellow_min, yellow_max)

    white_min = np.array([0, 0, 200], np.uint8)
    white_max = np.array([255, 30, 255], np.uint8)
    white_mask = cv2.inRange(img, white_min, white_max)

    filtered = img
    filtered[((yellow_mask == 0) & (white_mask == 0))] = 0
    color = cv2.cvtColor(filtered, cv2.COLOR_HSV2RGB)
    color1 = cv2.cvtColor(color, cv2.COLOR_RGB2GRAY)
    img1 = warper.warp(color1)
    left_fit, right_fit = polyfitter.polyfit(img1)
    #print(left_fit.shape)
    img2 = polydrawer.draw(video, left_fit, right_fit, warper.Minv)
    print(i)

    cv2.imshow("videos",img2)


    key = cv2.waitKey(5)
    if key == 27:
       break


video.release()
plt.destroyAllWindows()

