import cv2
import numpy as np


class Polydrawer:
    def draw(self, img, left_fit, right_fit, Minv):
        color_warp = np.zeros_like(img).astype(np.uint8)

        fity = np.linspace(0, img.shape[0] - 1, 100)
        left_fitx = left_fit[0] * fity ** 2 + left_fit[1] * fity + left_fit[2]
        right_fitx = right_fit[0] * fity ** 2 + right_fit[1] * fity + right_fit[2]
        midA = [(right_fitx[i] + left_fitx[i])/2 for i in range (len(left_fitx))]
        # Recast the x and y points into usable format for cv2.fillPoly()
        #pts_mid = np.array([np.transpose(np.vstack([midA, fity]))])
        #pts_left = np.array([np.transpose(np.vstack([left_fitx, fity]))])
        #pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, fity])))])
        midx=np.array(midA, dtype=np.int32)
        midy=np.array(fity, dtype=np.int32)
       
        #pts = np.hstack((pts_mid, pts_right))
        #pts = np.array(pts_mid, dtype=np.int32)
       # print(pts[(1-1)], "mid")
        
        #print(midy)
        for x in range(0,99):
            #print(x) 
            cv2.line(color_warp,(midx[x],midy[x]),(midx[x+1],midy[x+1]),(255,0,0),10)
        #  Warp the blank back to original image space using inverse perspective matrix (Minv)
        newwarp = cv2.warpPerspective(color_warp, Minv, (img.shape[1], img.shape[0]))
        # Combine the result with the original image
        result = cv2.addWeighted(img, 1, newwarp, 0.3, 0)

        return result
