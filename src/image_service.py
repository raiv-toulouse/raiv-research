#!/usr/bin/env python
import cv2
import rospy
from cv_bridge import CvBridge
from raiv_libraries.srv import rgb_service, rgb_serviceResponse, rgb_serviceRequest
from raiv_libraries.srv import depth_service, depth_serviceResponse, depth_serviceRequest
from sensor_msgs.msg import Image
import numpy as np

class ImageService:

    def __init__(self):
        #Declaration of all the node names
        rospy.init_node('image_distribution', anonymous = True)
        self.rgb_node_name = '/camera/color/image_raw'
        self.depth_node_name = '/camera/aligned_depth_to_color/image_raw'

        #Declaration of our 2 services, one for rgb image and the other for the depth image
        self.rgb_service = rospy.Service('/rgb_service', rgb_service, self.rgb_distribution)
        self.depth_service = rospy.Service('/depth_service', depth_service, self.depth_distribution)

    #Function to normalize the images
    def normalization(self, image, bins=255):
        image_histogram, bins = np.histogram(image.flatten(), bins, density=True)
        cdf = image_histogram.cumsum()  # cumulative distribution function
        cdf = cdf / cdf[-1]  # normalize

        # use linear interpolation of cdf to find new pixel values
        image_equalized = np.interp(image.flatten(), bins[:-1], cdf)

        return image_equalized.reshape(image.shape), cdf

    #Function to transfer from ROS imgmsg to cv2 or the inverse. op = 1 is from cv2 to imgmsg, op = 0 from omgmsg to cv2
    def cv2_msg_transf(self, image, op = 1, encoding = 'passthrough'):
        bridge = CvBridge()
        if op == 1:
            image = bridge.cv2_to_imgmsg(image)
        else:
            image = bridge.imgmsg_to_cv2(image, desired_encoding = encoding)
        return image

    #Function to get a new rgb image and send it as a response
    def rgb_distribution(self, req):
        image_rgb = rospy.wait_for_message(self.rgb_node_name, Image)
        return rgb_serviceResponse(
            image = image_rgb
        )

    #Function to get a new depth image, process it and send it as a response
    def depth_distribution(self, req):
        #Get new image and transfer it from imgmsg to cv2
        image_depth = rospy.wait_for_message(self.depth_node_name, Image)
        image_depth = self.cv2_msg_transf(image_depth, op = 0)

        #Check if ksize param. passed, if so we apply a MedianBlur on the depth image with the Kernel size equal to the param ksize
        if req.ksize != 0:
            image_depth = 255 - image_depth * 255
            image_depth = image_depth.astype(np.uint8)
            if req.ksize % 2 == 0:
                req.ksize += 1
            image_depth = cv2.medianBlur(image_depth, req.ksize)

        #Check if param 'normalization' is 1, if so we apply a normalization to the image, if not, the image is untouched.
        if req.normalization == 1:
            image_depth = self.normalization(image_depth)[0]
        #Transfer the image back to imgmsg from cv2
        image_depth = self.cv2_msg_transf(image_depth, op = 1)

        #Send the depth image as a response
        return depth_serviceResponse(
            image = image_depth
        )

if __name__ == '__main__':
    images = ImageService()
    rate = rospy.Rate(30)
    req = depth_serviceRequest(ksize = 0, normalization = 0)
    while not rospy.is_shutdown():
        key = cv2.waitKey(1)

        rgb = images.rgb_distribution(()).image
        rgb = images.cv2_msg_transf(rgb, op = 0, encoding ='bgr8')

        depth = images.depth_distribution(req).image
        depth = images.cv2_msg_transf(depth, op = 0)
        print(depth)
        cv2.imshow('Rgb', rgb)
        cv2.imshow('Depth', depth)
        rate.sleep()