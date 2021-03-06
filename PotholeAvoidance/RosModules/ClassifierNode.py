#!/usr/bin/env python

"""
A ROS module that will subscribe to the raspberry pi camera 
and publish a boolean flag 0 or 1 indicating the presence of
a pothole in frame. 
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



class pothole_classifier:
    def __init__(self):
        """
        A constructor for an object that will subscribe to the
        relevant topics and apply processing. 
        """
        self.pub = rospy.Publisher("chatter",
            Bool, queue_size=5)

        self.sub = rospy.Subscriber("/raspicam_node/image/compressed",
            CompressedImage, self.callback, queue_size=5)
        
        # make tensorflow resources available 
        self.session = tf.Session()
        self.graph = tf.get_default_graph()
        with self.graph.as_default():
            with self.session.as_default():
                print("Got the graph and session...")
                self.model = load_model('classification_model_2020-03-05_23-00-40demo.h5')
                print("Loaded model...")

        # images will be resized
        self.size = (128, 128)

        print(self.model.summary())
        
    def callback(self, ros_data):
        # input
        arr = np.fromstring(ros_data.data, np.uint8)
        image = cv2.imdecode(arr, cv2.IMREAD_COLOR)

        image = cv2.resize(image, self.size)
        image = np.expand_dims(image, axis=0)
        # print("Image: ")
        # print(type(image))
        # print(image.shape)
        

        # run our model on the image
        with self.graph.as_default():
            with self.session.as_default():
                [result] = self.model.predict(image)
            # set_session(sess)
            # predictions = self.model.predict(image)
        # print("Made some predictions...")
        # print("Reducing image dim...")
        # image = image[0]
        # print(image.shape)
        # colour = (0, 0, 255)
        # thickness = 2

        print("Model output: ", result)
        
        # Display the classification results on the image
        # if result:
        #     cv2.putText(image, "yes", (10,10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
        # else:
        #     cv2.putText(image, "no", (10,10), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2)

        # output
        self.pub.publish(result)


def main(args):
    # start the ros node
    pc = pothole_classifier()
    rospy.init_node('pothole_classifier', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down pothole classifier node")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
