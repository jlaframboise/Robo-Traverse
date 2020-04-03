#!/usr/bin/env python

"""
A ROS module that will subscribe to the raspberry pi camera 
and apply a CNN which will regress a bounding box locating
the pothole in an image from the camera. It will output 
graphical results and publish the bounding box coordinates.  
"""

import sys, time, math, os

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as pat
import pandas as pd
import cv2
import xml.etree.ElementTree as ET
import roslib
import rospy

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool

import tensorflow as tf
from tensorflow.keras.layers import *
from tensorflow.keras.models import *
from tensorflow.keras.optimizers import *
from tensorflow.keras.utils import *
from tensorflow.keras.backend import set_session

from sklearn.model_selection import train_test_split



class image_processor:
    def __init__(self):
        # load camera calibration
        self.cameraCal = np.load('CalibrationMatrix-02-29.npy')

        # start the publisher and subscribers
        self.pub = rospy.Publisher("/output/image_raw/compressed",
            CompressedImage, queue_size=5)

        self.sub = rospy.Subscriber("/raspicam_node/image/compressed",
            CompressedImage, self.callback, queue_size=5)

        self.classifier_sub = rospy.Subscriber('chatter', Bool, self.check_pothole)

        
        # make tensorflow available
        self.session = tf.Session()
        self.graph = tf.get_default_graph()
        with self.graph.as_default():
            with self.session.as_default():
                print("Got the graph and session...")
                self.model = load_model('model_2020-03-05_21-40-27-demo1.h5')
                print("Loaded model...")

        
        self.size = (128, 128)

        print(self.model.summary())
        
    def check_pothole(self, pothole_data):
        self.pothole_present = pothole_data.data

    def callback(self, ros_data):
        # input
        arr = np.fromstring(ros_data.data, np.uint8)
        image = cv2.imdecode(arr, cv2.IMREAD_COLOR)

        image = cv2.resize(image, self.size) / 255.0
        image = np.expand_dims(image, axis=0)
        # print("Image: ")
        # print(type(image))
        # print(image.shape)
        

        if(self.pothole_present):
            # run our model on the image
            with self.graph.as_default():
                with self.session.as_default():
                    [[x,y,w,h]] = self.model.predict(image)
                # set_session(sess)
                # predictions = self.model.predict(image)
            print("Made some predictions...")
            print("Reducing image dim...")
            image = image[0] * 255.0
            print(image.shape)
            colour = (0, 0, 255)
            thickness = 1

            print("Model output")
            print((x,y,w,h))

            # scale output to pixels from ratios
            x = int(x*self.size[0])
            y = int(y*self.size[1])
            w = int(w*self.size[0])
            h = int(h*self.size[1])

            print("Scaled output")
            print((x,y,w,h))

            cv2.rectangle(image, (x,y), (x+w,y+h), colour, thickness)

            # to scale points to what is required as input for cameraCal
            x = x*5
            y = y*3.75
            w = w*5
            h = h*3.75

            # convert bounding box to real world points
            a = np.array([
                        [x, y],
                        [x+w, y+h], 
                        ], dtype='float32')
            # a = np.array([[436, 295], [185, 296], [400, 279], [220, 280], [314, 286], [470, 279], [149, 281],
            #      [519, 297], [45, 297], [134, 383], [545, 381], [468, 332], [200, 332], 
            #      [328, 277], [461, 288], [145, 293], [248, 311], [379, 360]], dtype='float32')
            a = np.array([a])
            realPts = cv2.perspectiveTransform(a, self.cameraCal)

            print("real point", realPts)
            
        else:
            image = image[0] * 255.0
            print("No pothole")
        # output
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', image)[1]).tostring()

        self.pub.publish(msg)


def main(args):

    ip = image_processor()
    rospy.init_node('image_processor', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down pothole localizer node")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
