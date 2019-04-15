import cv2
import numpy as np


class Polyfitter:
    def __init__(self):
        self.left_fit = None
        self.right_fit = None
        self.leftx = None
        self.rightx = None

    def polyfit(self, img):
        if self.left_fit is None or self.right_fit is None:
            return self.polyfit_sliding(img)
        else:
           nonzero = img.nonzero()
           nonzeroy = np.array(nonzero[0])
           nonzerox = np.array(nonzero[1])
           margin = 100
           
           left_lane_inds = (
        (nonzerox > (self.left_fit[0] * (nonzeroy ** 2) + self.left_fit[1] * nonzeroy + self.left_fit[2] - margin)) & (
        nonzerox < (self.left_fit[0] * (nonzeroy ** 2) + self.left_fit[1] * nonzeroy + self.left_fit[2] + margin)))
           right_lane_inds = (
        (nonzerox > (self.right_fit[0] * (nonzeroy ** 2) + self.right_fit[1] * nonzeroy + self.right_fit[2] - margin)) & (
        nonzerox < (self.right_fit[0] * (nonzeroy ** 2) + self.right_fit[1] * nonzeroy + self.right_fit[2] + margin)))

           self.leftx = nonzerox[left_lane_inds]
           lefty = nonzeroy[left_lane_inds]
           self.rightx = nonzerox[right_lane_inds]
           righty = nonzeroy[right_lane_inds]
           self.left_fit = np.polyfit(lefty, self.leftx, 2)
           self.right_fit = np.polyfit(righty, self.rightx, 2)

           return self.left_fit, self.right_fit

    def polyfit_sliding(self, img):
        histogram = np.sum(img[int(img.shape[0] / 2):, :], axis=0)
        out_img = np.dstack((img, img, img)) * 255
        midpoint = np.int(histogram.shape[0] / 2)
        leftx_base = np.argmax(histogram[:midpoint])
        rightx_base = np.argmax(histogram[midpoint:]) + midpoint

        nwindows = 9
        window_height = np.int(img.shape[0] / nwindows)
        nonzero = img.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        leftx_current = leftx_base
        rightx_current = rightx_base
        margin = 100
        minpix = 50
        left_lane_inds = []
        right_lane_inds = []

        for window in range(nwindows):
            win_y_low = img.shape[0] - (window + 1) * window_height
            win_y_high = img.shape[0] - window * window_height
            win_xleft_low = leftx_current - margin
            win_xleft_high = leftx_current + margin
            win_xright_low = rightx_current - margin
            win_xright_high = rightx_current + margin
          #  cv2.rectangle(out_img, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high), (0, 255, 0), 2)
          #  cv2.rectangle(out_img, (win_xright_low, win_y_low), (win_xright_high, win_y_high), (0, 255, 0), 2)
            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xleft_low) & (
                nonzerox < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xright_low) & (
                nonzerox < win_xright_high)).nonzero()[0]
            left_lane_inds.append(good_left_inds)
           
            right_lane_inds.append(good_right_inds)
            if len(good_left_inds) > minpix:
                leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
            if len(good_right_inds) > minpix:
                rightx_current = np.int(np.mean(nonzerox[good_right_inds]))

        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)
        #cv2.imshow("videos out",out_img)
        self.leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds]
        self.rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]

        self.left_fit = np.polyfit(lefty, self.leftx, 2)
        self.right_fit = np.polyfit(righty, self.rightx, 2)
        
        return self.left_fit, self.right_fit

    
