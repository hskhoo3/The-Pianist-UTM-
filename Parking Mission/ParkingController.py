import rospy
import cv2
import numpy as np
from enum import Enum
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist

rospy.Subscriber('/sign_detected/parking',Bool,parkingMissionStarts)

class controlParking():
	def __init__(self):
		rospy.Subscriber('/scan',LaserScan,self.check_lidar_distance)
		rospy.Subscriber('/sonar/left',Range,self.check_left_sonar)
		rospy.Subscriber('/sonar/right',Range,self.check_right_sonar)
		rospy.Subscriber('/camera/image/compressed_fisheye',CompressedImage, self.line_trace,  queue_size = 1)

		self.turtlemove_pub = rospy.Publisher('/cmd_vel',Twist,queue_size = 1)

		lower_yellow=np.array([27,75,163])
		upper_yellow=np.array([35,163,225])
		lower_white=np.array([0,0,200])
		upper_white=np.array([180,15,255])

		self.whiteLineDetected = False
		self.yellowLineDetected = False
		self.higher_obstacle = False
		self.lower_obstacle = False

		self.parkingState = Enum('parkingState', 'reverseCurve reverseStraight parkingFinish')

	def line_trace(self,eyes):
        np_arr = np.fromstring(eyes.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        #convert scalar to array
		cv2.imshow('frame',frame)

		hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
	    mask_white=cv2.inRange(hsv,lower_white,upper_white)
		white=cv2.bitwise_and(frame,frame,mask=mask_white)

		hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
	    mask_yellow=cv2.inRange(hsv,lower_yellow,upper_yellow)
		yellow=cv2.bitwise_and(frame,frame,mask=mask_yellow)

		#cv2.line(frame,(165,175),(475,175),(253,244,8),2)
		#cv2.line(frame,(165,235),(475,235),(253,244,8),2)
		#cv2.line(frame,(165,175),(165,235),(253,244,8),2)
		#cv2.line(frame,(475,175),(475,235),(253,244,8),2)

		gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        cv2.imshow('gray', gray)
        ##ROI=gray[180:230,170:470]
        ROI=gray[int(gray.shape[0]/2):,:]
        ROI=cv2.GaussianBlur(ROI,(21,21),0)
        thr=cv2.adaptiveThreshold(ROI,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,11,2)
        blur=cv2.medianBlur(thr,9)	
        edge=cv2.Canny(blur,0,360)
        yellow_edge=edge[:,:]
        white_edge=edge[int(edge.shape[0]/2):,int(edge.shape[1]/2):]
        white_blur=blur[int(edge.shape[0]/2):,int(edge.shape[1]/2):]
        yellow_lines=cv2.HoughLines(yellow_edge,1,np.pi/180,30)
        white_lines=cv2.HoughLines(white_edge,1,np.pi/180,30)   
        #cv2.imshow('road',white_edge)
        #print(white_lines)
        if white_lines is not None:
            i=len(white_lines)-1
            x=-1
            num=1
            no=1
            while x < i:
                for rho,theta in white_lines[x]:
                    a = np.cos(theta)
                    b = np.sin(theta)
                    x0 = a*rho
                    y0 = b*rho
                    x1 = int(x0 + 1000*(-b))
                    y1 = int(y0 + 1000*(a))
                    x2 = int(x0 - 1000*(-b))
                    y2 = int(y0 - 1000*(a))
                    x+=1
                    length=math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
                    degree=np.arctan2(y2-y1,x2-x1)*180/np.pi
                    no+=1
                    #rr=cv2.line(white_blur,(x1,y1),(x2,y2),(0,0,255),3) #+320+180
                    #print(no, ',',length)
                    #print('Degree=',degree)
                    if length > 1999 and degree<5 and degree>-20:
                        
                        print(num,"point1:",x1,",",y1,'\n'"point2:",x2,",",y2,'\n'"length:",length)
                        print('Degree=',degree)
                        print('yes')
                        #self.whiteLineDetected=True
                        rr = cv2.line(white_blur,(x1,y1),(x2,y2),(0,0,255),3) #+320+180
    
                        num+=1
                        
                        break
                    else:
                        continue
            
##    elif:
##        print('see nothing')
        cv2.imshow('sohai', rr)
        cv2.imshow('lusohai',frame)
                
		
		if yellow_lines is not None:
			yellow_lines=[l[0] for l in L_lines]
			for rho,theta in L_lines:
		    		a = np.cos(theta)
		    		b = np.sin(theta)
		    		x0 = a*rho
		    		y0 = b*rho
		    		x1 = int(x0 + 1000*(-b))
		    		y1 = int(y0 + 1000*(a))
		    		x2 = int(x0 - 1000*(-b))
		    		y2 = int(y0 - 1000*(a))
				degree=np.arctan2(y2-y1,x2-x1)*180/np.pi
				# modified degree to fit the view
				if degree<5 and degree>-5:
					self.yellowLineDetected=True	
					cv2.line(frame,(x1+170,y1+180),(x2+170,y2+180),(0,100,100),3)
					break
				else:
					continue


			if self.whiteLineDetected == True:
				self.turlemove(0.06,0.00) 

			else:
				if self.higher_obstacle or self.lower_obstacle == True:
					turlemove(0.06,0.00)
				else:
					self.currentParkingState = self.parkingState.reverseCurve
					self.parkingStarts(self.currentParkingState)

			if self.yellowLineDetected == True:
				self.currentParkingState = self.parkingState.reverseStraight
				self.parkingStarts(self.currentParkingState)
	
	def check_lidar_distance(self,lidar):

		lidarRightAngle = lidar.ranges[45:135]

		for dist in lidarRightAngle:
			if dist > 0.12 and dist < 0.6:
				self.higher_obstacle=True

	def check_left_sonar(self,left_sonar)

		sonarLeftDist=left_sonar.range

		if sonarLeftDist < 0.3:
			lower_obstacle=True

	def check_right_sonar(self,right_sonar)

		sonarRightDist=right_sonar.range

		if sonarRightDist < 0.3:
			lower_obstacle=True
		
	def parkingStarts(self,parkingStateMsg):
	
		if parkingStateMsg == self.parkingState.reverseCurve		
			turtlemove(-0.11,0.7)
			startTimeCount = rospy.get_rostime()

		elif parkingStateMsg == self.parkingState.reverseStraight
			endTimeCount = rospy.get_rostime()
			turtlemove(-0.1,0)
			rospy.sleep(rospy.Duration(1.7))
			turtlemove(0,0)
			rospy.sleep(time)
			turtlemove(0.1,0)
			rospy.sleep(rospy.Duration(1.7))
			turtlemove(0.11,-0.7)
			rospy.sleep(endTimeCount-startTimeCount)
			turtlemove(0,0)
			self.currentParkingState = parkingFinish	
			
	def turtlemove(self,linear,angular):

		twist=Twist()
		twist.linear.x=linear
		twist.angular.z=angular
		turtlemove_pub.publish(twist)

	def main(self)
		rospy.spin()

if __name__ == '__main__':
	rospy.init_node('parking_ctrl_node')

	def parkingMissionStarts(sign_detection):
   		if sign_detection == True:
			parking_controller = controlParking()
			parking_controller.main()				
