#!/usr/bin/env python3


import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageViewerNode(Node):
    def __init__(self):
        super().__init__('image_viewer_node')
        self.bg_subtractor = cv2.createBackgroundSubtractorMOG2(detectShadows = True) # exclude shadow areas from the objects you detected
        # self.bg_subtractor = cv2.createBackgroundSubtractorKNN(detectShadows = True) # detectShadows=True : exclude shadow areas from the objects you detected

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # Change to your actual image topic
            self.image_callback,
            10
        )

    def image_callback(self, msg):
        try:
            # Convert to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # Every frame is used both for calculating the foreground mask and for updating the background. 
            foreground_mask = self.bg_subtractor.apply(cv_image)

            # threshold if it is bigger than 240 pixel is equal to 255 if smaller pixel is equal to 0
            # create binary image , it contains only white and black pixels
            ret , treshold = cv2.threshold(foreground_mask.copy(), 120, 255,cv2.THRESH_BINARY)
            
            #  dilation expands or thickens regions of interest in an image.
            dilated = cv2.dilate(treshold,cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3)),iterations = 2)
            
             # find contours 
            contours, hier = cv2.findContours(dilated,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # check every contour if are exceed certain value draw bounding boxes
            for contour in contours:
                # if area exceed certain value then draw bounding boxes
                if cv2.contourArea(contour) > 50:
                    (x,y,w,h) = cv2.boundingRect(contour)
                    cv2.rectangle(cv_image, (x,y), (x+w, y+h), (255, 255, 0), 2)

            # Show image using OpenCV
            cv2.imshow('Camera Image', cv_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ImageViewerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
