#!/usr/bin/env python
import roslib; roslib.load_manifest('Daneel')
import rospy

import sensor_msgs.msg
import cv_bridge
import cv
import datetime
import threading

import pydrone.srv
import pydrone.ARDrone as ARDrone
import pydrone.msg as msg


class DroneController:
    """
    The drone is keyboard-controlled for movement. Clicking in the image
    window will give the RGB values at that point.
    """

    def __init__(self):
        """ constructor; setting up data """
        self.init_drone_parameters()
        self.bridge = cv_bridge.CvBridge()
        self.counter = 0
        self.image = cv.CreateImage((640, 480), 8, 3)
        self.rgb = (0,0,0)
        
        self.bf = BlobFinder()
        
        self.t_center = (160, 120)
        self.center_thresh = 30
        
        self.target_area = 9000
        self.area_thresh = 600

        #Target values (doubled image size):
        # big area = 180000
        # small area = 9000
        
        self.blob_threshold = 500
        
        cv.NamedWindow('image')
        cv.MoveWindow('image',320,0)
        cv.NamedWindow('threshold_1')
        cv.MoveWindow('threshold_1',320,480)
        cv.NamedWindow('threshold_2')
        cv.MoveWindow('threshold_2',960,0)
        
        self.make_slider_window()
        
        cv.SetMouseCallback('image', self.onMouse, None)
        
        
    def init_drone_parameters(self):
        """ drone-specific parameters: speed, camera, etc. """
        print "Constructing the drone software object..."
        self.drone = ARDrone.ARDrone()
        # Drone's motion speed
        self.drone.speed = 0.2
        self.drone.cam = 0 # 0 for forward, 1 for floor camera (can be changed
                      # in flight)
        self.drone.frequency = 8 # Drone's ultrasound frequency - 7 for 22.22
                            # Hz, 8 for 25 Hz
        self.drone.altmax_value = 10000 # Drone's maximum altitude between 500
                                   # and 5000 or 10000 for unlimited
        #print "Trying to connect to the drone..."
        self.drone.connect()
        self.drone.freq(self.drone.frequency)
        self.drone.altmax(self.drone.altmax_value)
        self.drone.feed(1)
        rospy.sleep(1)
        print "Drone initialized."

    def getPicture(self, data):
        """ gets called with each image from the drone """
        image = self.bridge.imgmsg_to_cv(data, "bgr8")
        cv.Resize(image, self.image)
        
        
    def getData(self, data):
        """ the drone's navigation and status data """
        data_dictionary = eval(data.data)
        self.battery_level = data_dictionary['battery']
        # also available:
        # 'phi', 'psi', 'num_frames', 'altitude', 'ctrl_state',
        # 'vx', 'vy', 'vz', 'theta'

    def onMouse(self,event,x,y,flags,param):
        """ mouse-click handler from OpenCV """
        if event == cv.CV_EVENT_LBUTTONDOWN and self.image != None:
            (b,g,r) = self.image[y,x]
            self.rgb = (r,g,b)
            (h,s,v) = self.bf.hsv[y,x]
            print "RGB values are", int(r), int(g), int(b)
            print "HSV values are", int(h), int(s), int(v)
            print "Hue = ", int(h)
            
    def make_slider_window(self):
        #Create slider window
        cv.NamedWindow('sliders')
        cv.MoveWindow('sliders', 0, 0)
        
        #Create sliders
        cv.CreateTrackbar('low_hue_1', 'sliders', self.bf.low_hue_1,\
                          181, self.change_low_hue_1)  
        cv.CreateTrackbar('high_hue_1', 'sliders', self.bf.high_hue_1,\
                          181, self.change_high_hue_1)
        cv.CreateTrackbar('low_hue_2', 'sliders', self.bf.low_hue_2,\
                          181, self.change_low_hue_2)  
        cv.CreateTrackbar('high_hue_2', 'sliders', self.bf.high_hue_2,\
                          181, self.change_high_hue_2) 
        cv.CreateTrackbar('low_sat', 'sliders', self.bf.low_sat,\
                          256, self.change_low_sat)  
        cv.CreateTrackbar('high_sat', 'sliders', self.bf.high_sat,\
                          256, self.change_high_sat)  
        
    #Functions for changing the slider values  
    def change_low_hue_1(self, new_threshold):
        self.bf.low_hue_1 = new_threshold
        
    def change_high_hue_1(self, new_threshold):
        self.bf.high_hue_1 = new_threshold
    
    def change_low_hue_2(self, new_threshold):
        self.bf.low_hue_2 = new_threshold
        
    def change_high_hue_2(self, new_threshold):
        self.bf.high_hue_2 = new_threshold
    
    def change_low_sat(self, new_threshold):
        self.bf.low_sat = new_threshold
        
    def change_high_sat(self, new_threshold):
        self.bf.high_sat = new_threshold
        
    def keyboardThread(self):
        """ the main keypress-handling thread to control the drone """
        call = self
        drone = self.drone
        airborne = False
        moving = False
        looking = False

        # this is the main loop for the keyboard thread
        while True:
            self.bf.image = self.image
            self.bf.find_blob()
            cv.ShowImage('image', self.bf.image)
            cv.ShowImage('threshold_1', self.bf.threshold_1)
            cv.ShowImage('threshold_2', self.bf.threshold_2)

            if self.counter == 99:
                print "blob area 1: ", self.bf.blob_area_1
                print "blob area 2: ", self.bf.blob_area_2
                print "center 1: ", self.bf.center_1
                print "center 2: ", self.bf.center_2
            self.counter = (self.counter + 1) % 100
            
            
            # get the next keypress
            c = cv.WaitKey(50)&255

            # handle that keypress
            if c == ord('q') or c == 27: # the Esc key is 27

                if looking:
                    print "quitting"
                    self.state = "quit"
                    looking = not looking
                    rospy.Timer(rospy.Duration(3), self.fsm, oneshot=True)
                print 'quit'
                drone.land()
                drone.halt()
                return
                
            elif c == 115: #115 = s
                print 'saving'
                f = open("thresholdsAR.txt","w")
                print >> f, "low_hue_1 = ", self.bf.low_hue_1
                print >> f, "high_hue_1 = ", self.bf.high_hue_1
                print >> f, "low_hue_2 = ", self.bf.low_hue_2
                print >> f, "high_hue_2 = ", self.bf.high_hue_2
                print >> f, "low_sat = ", self.bf.low_sat
                print >> f, "high_sat = ", self.bf.high_sat
                f.close()

            elif c == 108: #108 = l
                print "loading thresholdsAR.txt"
                f = open("thresholdsAR.txt","r")
                L = f.readlines()
                
                self.bf.low_hue_1 = int(L[0].split()[2])
                self.bf.high_hue_1 = int(L[1].split()[2])
                self.bf.low_hue_2 = int(L[2].split()[2])
                self.bf.high_hue_2 = int(L[3].split()[2])
                self.bf.low_sat = int(L[4].split()[2])
                self.bf.high_sat = int(L[5].split()[2])
                
                #Recreate the slider window
                cv.DestroyWindow('sliders')
                self.make_slider_window()
            
#            elif c == ord('\n'):   # newline for takeoff/landing
#                if not airborne:
#                    print 'take off'
#                    drone.takeoff()
#                    rospy.sleep(0.07)
#                else:
#                    print 'land'
#                    drone.land()
#                    rospy.sleep(0.07)
#                airborne = not airborne

            elif c == ord('e'):   # 'e' switches the video feed
                print 'switch video feed, current = ', drone.cam
                print 'new = ', (1 + drone.cam) % 4
                drone.cam = (1 + drone.cam) % 4
                drone.feed(drone.cam)

            elif c == ord('f'): # 'f' starts the find subroutine
                    drone.cam = 0
                    drone.feed(drone.cam)     
                    looking = True
                    self.state = "start"
                    self.fsm()

#            elif airborne:   # the different direction controls
#                if not moving:
#                    if c == ord('a'):
#                        print 'left'
#                        moving = True
#                        drone.moveRight(-drone.speed)
#                    elif c == ord('d'):
#                        print 'right'
#                        moving = True
#                        drone.moveRight(drone.speed)
#                    elif c == ord('w'):
#                        print 'forward'
#                        moving = True
#                        drone.moveForward(drone.speed)
#                    elif c == ord('s'):
#                        print 'backward'
#                        moving = True
#                        drone.moveForward(-drone.speed)
#                    elif c == ord('j'):
#                        print 'turn left'
#                        moving = True
#                        drone.moveRotate(-2*drone.speed)
#                    elif c == ord('l'):
#                        print 'turn right'
#                        moving = True
#                        drone.moveRotate(2*drone.speed)
#                    elif c == ord('i'):
#                        print 'up'
#                        moving = True
#                        drone.moveUp(drone.speed)
#                    elif c == ord('k'):
#                        print 'down'
#                        moving = True
#                        drone.moveUp(-drone.speed)
#                else:
#                    moving = False
#                    drone.hover()

    def fsm(self, timer_event=None):
        print "now in state:", self.state
        drone = self.drone
        sleep_time = 0.6

        if self.state = "quit":
            drone.land()
            return

        if self.state = "start":
            drone.takeoff()
            self.state = "find"
            sleep_time = 8

        elif self.state = "find":
            # hover to decide next step
            drone.hover()
            
            # if we can see the blob
            if self.bf.blob_area_1 > blob_threshold:
                # if the small blob is the right size we're there
                if abs(self.bf.blob_area_2 - self.target_area) < self.area_thresh:
                    self.state = "at marker"
                elif self.bf.center_1[0] < self.t_center[0] - self.center_thresh:
                    self.state = "move left"
                elif self.bf.center_1[0] > self.t_center[0] + self.center_thresh:
                    self.state = "move right"
                elif self.bf.center_1[1] < self.t_center[1] - self.center_thresh:
                    self.state = "move up"
                elif self.bf.center_1[1] > self.t_center[1] + self.center_thresh:
                    self.state = "move down"
                elif self.bf.blob_area_1 > self.target_area + self.area_thrseh:
                    self.state = "move backward"
                else
                    self.state = "move forward"
            else:
                self.state = "no marker"
            sleep_time = 0.2

        else:
            if self.state = "at marker":
                # if we're at the marker, turn left about 90 degrees
                drone.moveRotate(-drone.speed)
                sleep_time = 2

            elif self.state = "no marker":
                # if we can't see the marker, turn left until we find it
                drone.moveRotate(-drone,speed)

            elif self.state = "move left":
                drone.moveRight(-drone,speed)
            
            elif self.state = "move up":
                drone.moveUp(drone,speed)
         
            elif self.state = "move down":
                drone.moveUp(-drone,speed)

            elif self.state = "move backward":
                drone.moveForward(-drone,speed)

            elif self.state = "move forward":
                drone.moveForward(drone,speed)
        
            self.state = "find"

        rospy.Timer(rospy.Duration(sleep_time), self.fsm, oneshot=True)
        return

def imageSubscriber(call):
    rospy.Subscriber('arimage', sensor_msgs.msg.Image, call.getPicture)
    rospy.spin()

def dataSubscriber(call):
    rospy.Subscriber('arnavdata', msg.NavData, call.getData)
    rospy.spin()

class BlobFinder:
    def __init__(self):

        self.high_hue_1 = 181
        self.low_hue_1 = 0
        self.high_hue_2 = 181
        self.low_hue_2 = 0
        self.low_sat = 0
        self.high_sat = 256        

        self.blob_area_1 = None
        self.blob_area_2 = None
        
        self.center_1 = None
        self.center_2 = None
        
        self.image = None
        self.hsv = None
        self.threshold_1 = None
        self.threshold_2 = None
                           
    def find_blob(self):       
        
        #Create the thresholded image
        self.create_image()
        
        #Find blob regions, draw a rectangle around the largest and a circle at
        #its center.
        self.find_regions()
        
    def create_image(self):
        
        #Find the size of the image
        self.size = cv.GetSize(self.image)
        
        #Convert to HSV
        self.hsv = cv.CreateImage(self.size, 8, 3)
        cv.CvtColor(self.image, self.hsv, cv.CV_BGR2HSV)
        
        #Create images for each channel
        hue = cv.CreateImage(self.size, 8, 1) 
        sat = cv.CreateImage(self.size, 8, 1)
        val = cv.CreateImage(self.size, 8, 1)
        
        hue_threshed_1 = cv.CreateImage(self.size, 8, 1)
        hue_threshed_2 = cv.CreateImage(self.size, 8, 1)

        sat_threshed = cv.CreateImage(self.size, 8, 1)

        self.threshold_1 = cv.CreateImage(self.size, 8, 1)
        self.threshold_2 = cv.CreateImage(self.size, 8, 1)

        #Split the image up into channels, saving them in their respective image
        cv.Split(self.hsv, hue, sat, val, None)
        
        #Threshold the images based on the slider values
        cv.InRangeS(hue, self.low_hue_1, self.high_hue_1, hue_threshed_1)
        cv.InRangeS(hue, self.low_hue_2, self.high_hue_2, hue_threshed_2)  

        cv.InRangeS(sat, self.low_sat, self.high_sat, sat_threshed)

        cv.Mul(hue_threshed_1, sat_threshed, self.threshold_1)
        cv.Mul(hue_threshed_2, sat_threshed, self.threshold_2)                 
        
        #Erode and Dilate shave off and add edge pixels respectively
        cv.Erode(self.threshold_1, self.threshold_1, iterations = 1)
        cv.Dilate(self.threshold_1, self.threshold_1, iterations = 1)
        
        cv.Erode(self.threshold_2, self.threshold_2, iterations = 1)
        cv.Dilate(self.threshold_2, self.threshold_2, iterations = 1)

        #Save the new threshold images
        #self.threshold_1 = threshed_1
        #self.threshold_2 = threshed_2
        
    def find_regions(self):
    
        #Create a copy image of thresholds then find contours on that image
        storage_1 = cv.CreateMemStorage(0)
        storage_2 = cv.CreateMemStorage(0)
        copy_1 = cv.CreateImage(self.size, 8, 1)
        copy_2 = cv.CreateImage(self.size, 8, 1)
        cv.Copy( self.threshold_1, copy_1 )
        cv.Copy( self.threshold_2, copy_2 )
        contours_1 = cv.FindContours(copy_1, storage_1, cv.CV_RETR_EXTERNAL,\
                                   cv.CV_CHAIN_APPROX_SIMPLE)
        contours_2 = cv.FindContours(copy_2, storage_2, cv.CV_RETR_EXTERNAL,\
                                   cv.CV_CHAIN_APPROX_SIMPLE)
        
        if len(contours_1)>0:
            box_1 = self.find_largest_contour(contours_1)
            
            self.blob_area_1 = (box_1[2][0] - box_1[0][0])*(box_1[2][1] - box_1[0][1])
            
            #Draw the box, 
            cv.PolyLine(self.image,[[box_1[0], box_1[1], box_1[2], box_1[3]]],\
                        1, cv.RGB(255, 0, 0))
            
            #Draw the circle
            self.center_1 = ((box_1[0][0] + box_1[2][0])/2, (box_1[2][1] + box_1[0][1])/2)
            
            cv.Circle(self.image,(self.center_1[0], self.center_1[1]), 10, cv.RGB(255, 0, 0),\
                      thickness=1, lineType=8, shift=0)
            
            #Draw the contours
            cv.DrawContours(self.image, box_1[4], cv.RGB(255,0,0),
                            cv.RGB(255, 0, 0), 1, thickness=2, lineType=8,
                            offset=(0, 0))
        
        if len(contours_2) > 0:
            box_2 = self.find_largest_contour(contours_2)
            
            self.blob_area_2 = (box_2[2][0] - box_2[0][0])*(box_2[2][1] - box_2[0][1])   
                                    
            cv.PolyLine(self.image,[[box_2[0], box_2[1], box_2[2], box_2[3]]],\
                        1, cv.RGB(0, 255, 0))
                        
            self.center_2 = ((box_2[0][0] + box_2[2][0])/2, (box_2[0][1] + box_2[2][1])/2)
            
            cv.Circle(self.image,(self.center_2[0], self.center_2[1]), 10, cv.RGB(0, 255, 0),\
                      thickness=1, lineType=8, shift=0)
                      
            cv.DrawContours(self.image, box_2[4], cv.RGB(0,255,0),
                            cv.RGB(0, 255, 0), 1, thickness=2, lineType=8,
                            offset=(0, 0))
                                                                      
    def find_largest_contour(self, contours):                            
        #Find the largest contour
        biggest = contours
        biggestArea=cv.ContourArea(contours)
        while contours!=None:
            nextArea=cv.ContourArea(contours)
            if biggestArea < nextArea:
                biggest = contours
                biggestArea = nextArea
            contours=contours.h_next()

        after_biggest = biggest.h_next()
        after_biggest = None
        
        #Get a bounding rectangle for the largest contour
        br =cv.BoundingRect(biggest,update=0) 
        
        #Find the middle of it
        circle_x = br[0]+br[2]/2
        circle_y = br[1]+br[3]/2
        
        #Find the coordinates for each corner of the box
        box_tl = (br[0],br[1])
        box_tr = (br[0]+br[2],br[1])
        box_bl = (br[0],br[1]+br[3])
        box_br = (br[0]+br[2],br[1]+br[3])
        
        return box_tl, box_bl, box_br, box_tr, biggest
            
if __name__ == '__main__':
    '''
    Main driver function for the drone.
    '''
    # Setup stuff
    rospy.init_node('drone')

    controller = DroneController()

    rospy.Subscriber('arimage', sensor_msgs.msg.Image, controller.getPicture)
    rospy.Subscriber('arnavdata', msg.NavData, controller.getData)

    # Display initial message
    print 'Controls (all lower case):\n'
    print 'RETURN takes off and lands the drone.'
    print 'The WASD keys move the drone at a given height and orientation.'
    print 'I/K moves the drone up/down.'
    print 'J/L turns the drone left/right.'
    print 'Q halts the drone and quits the program.'

    rospy.sleep(1)
    
    # start the main loop, the keyboard thread:
    controller.keyboardThread()            
