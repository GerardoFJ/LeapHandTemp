#!/usr/bin/env python

"""@file MonochromeToColorMatrixVisualizer.py

@author Selim Chefchaouni Moussaoui
@brief Tactile matrix visualization with chosen colormap
@date 2022-08-23

@copyright Copyright (c) AIST 2022

@details
Subscribes to MONO8 cv2 image and transforms it to BRG8 cv2 image
with chosen colomap.

Sample usage:
rosrun tactile_info_framework MonochromeToColorMatrixVisualizer.py \
_rate:=10 _subtopic:="/matrix_raw" _pubtopic:="matrix_colored" \
_colormap:="gnuplot2"

@param  _rate       ROS node frequency (in Hz)
@param  _subtopic   Name of the sensor_msgs/Image topic the node subscribes to
@param  _pubtopic   Name of the sensor_msgs/Image topic the node publishes to
@param  _colormap   The colormap of the output image. List of available maps at
                    https://matplotlib.org/stable/tutorials/colors/colormaps.html
"""


import numpy as np
import rospy
import matplotlib.cm as cm
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from PIL import Image as PILImage


class MonochromeToColorMatrixVisualizer:
    """
    Tactile matrix visualization with chosen colormap
    """

    def __init__(self):
        """
        Constructor for MonochromeToColorMatrixVisualizer
        """

        # init ROS node
        rospy.init_node('MonochromeToColorMatrixVisualizer', anonymous=True)

        # get parameters
        self.rate = float(rospy.get_param('~rate', '10'))
        self.sub_topic_name = rospy.get_param('~subtopic', 'filtered')
        self.pub_topic_name = rospy.get_param('~pubtopic', 'filtered')
        self.colormap = rospy.get_param('~colormap', 'colormap')

        # init elements
        self.sub = rospy.Subscriber(self.sub_topic_name, Image, self.image_callback, queue_size=10)
        self.pub = rospy.Publisher(self.pub_topic_name, Image, queue_size=10)
        self.rate = rospy.Rate(self.rate)

        self.pub_msg = Image()
        self.bridge = CvBridge()

        try:
            self.colormap = getattr(cm, self.colormap)
        except Exception:
            self.colormap = cm.gist_earth # default colormap


    def image_callback(self, msg):
        """
        Callback for image subscriber. Transforms the MONO8 cv2 image
        to BRG8 cv2 image with chosen colomap.
        """

        input_image = self.bridge.imgmsg_to_cv2(msg, "mono8")
        output_image = PILImage.fromarray(np.uint8(self.colormap(input_image)*255))

        image_cv = cv2.cvtColor(np.array(output_image), cv2.COLOR_RGB2BGR)
        self.pub_msg = self.bridge.cv2_to_imgmsg(image_cv, "bgr8")


    def run(self):
        """
        Run the node
        """
        while not rospy.is_shutdown():
            self.pub.publish(self.pub_msg)
            self.rate.sleep()


# script execution
if __name__ == '__main__':
    mv = MonochromeToColorMatrixVisualizer()
    try:
        mv.run()
    except KeyboardInterrupt:
        print("Shutting down")
