#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue May  2 00:21:33 2023

@author: wiyagi
Modified by Muhammad Labiyb Afakh
"""

#import datetime

from ultralytics import YOLO
import cv2
import rospy
from std_msgs.msg import Int8
import numpy as np

def most_frequent(List):
    unique, counts = np.unique(List, return_counts=True)
    index = np.argmax(counts)
    return unique[index]
List = []
i = 0
filtered = 0 
if __name__ == '__main__' :
    pub =  rospy.Publisher('counter', Int8, queue_size=10)
    rospy.init_node('monitor', anonymous=True)
    rate = rospy.Rate(100)
    model = YOLO('yolov8s.pt')
    model.conf = 0.8
    model.iou = 0.9
    #video_cap = cv2.VideoCapture("test.mp4")
    video_cap = cv2.VideoCapture(0)
    video_cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    video_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)


    while not rospy.is_shutdown():
        try:
            # start time to compute the fps
            #start = datetime.datetime.now()

            ret, frame = video_cap.read()
            # if there are no more frames to process, break out of the loop
            # if not ret:
            #     break
            results = model.predict (frame, classes=[0] ,verbose=False, show=True, conf = 0.8)
            result = results[0]
            hcount = len(result.boxes)
            List.append(hcount)
            
            i = i + 1
            if i==10:
                filtered = most_frequent(List)
                i = 0
                List.clear()

            if cv2.waitKey(1) & 0xFF == ord('q'):
                cv2.destroyAllWindows()
                break

            pub.publish(filtered)
            rate.sleep()

        except rospy.ROSInterruptException:
            pass

    video_cap.release()
