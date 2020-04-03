#!/usr/bin/env python

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

import tensorflow as tf
from tensorflow.keras.layers import *
from tensorflow.keras.models import *
from tensorflow.keras.optimizers import *
from tensorflow.keras.utils import *
from tensorflow.keras.backend import set_session

from sklearn.model_selection import train_test_split



class image_processor:
    def __init__(self):
        self.cameraCal = np.load('calibrationmatrix0206.npy')
        self.pub = rospy.Publisher("/output/image_raw/compressed",
            CompressedImage, queue_size=5)

        self.sub = rospy.Subscriber("/raspicam_node/image/compressed",
            CompressedImage, self.callback, queue_size=5)
        
        self.session = tf.Session()
        self.graph = tf.get_default_graph()
        with self.graph.as_default():
            with self.session.as_default():
                print("Got the graph and session...")
                self.model = load_model('model_2020-01-30_19-11-26.h5')
                print("Loaded model...")

        # self.model = myModel
        
        self.size = (128, 128)

        print(self.model.summary())
        
    def callback(self, ros_data):
        # input
        arr = np.fromstring(ros_data.data, np.uint8)
        image = cv2.imdecode(arr, cv2.IMREAD_COLOR)

        image = cv2.resize(image, self.size)
        image = np.expand_dims(image, axis=0)
        print("Image: ")
        print(type(image))
        print(image.shape)
        

        # run our model on the image
        with self.graph.as_default():
            with self.session.as_default():
                [[x,y,w,h]] = self.model.predict(image)
            # set_session(sess)
            # predictions = self.model.predict(image)
        print("Made some predictions...")
        print("Reducing image dim...")
        image = image[0]
        print(image.shape)
        colour = (0, 0, 255)
        thickness = 2

        print("Model output")
        print((x,y,w,h))

        x = 0.1
        y = 0.7
        w = 0.4
        h = 0.1

        x = int(x*self.size[0])
        y = int(y*self.size[0])
        w = int(w*self.size[0])
        h = int(h*self.size[0])

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
        # a = np.array([[13, 477], [640, 480], [101, 331],[225, 332], [347, 335], 
        #       [478, 336], [611, 339], [189, 300], [268, 301], [350, 302], 
        #       [435, 304], [524, 305], [107, 286], [168, 286], [228, 286], 
        #       [290, 286], [351, 288], [415, 287], [480, 288], [545, 289]], dtype='float32')
        a = np.array([a])
        realPts = cv2.perspectiveTransform(a, self.cameraCal)
        realPts = realPts*0.0254

        # output
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', image)[1]).tostring()

        self.pub.publish(msg)

        print("real point", realPts)


def main(args):

    # global sess
    # global graph
    
    # global graph 
    # graph = tf.get_default_graph()

    ip = image_processor()
    rospy.init_node('image_processor', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down pothole localizer node")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)