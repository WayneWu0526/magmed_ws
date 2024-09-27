#!/usr/bin/env python3

import rospy
import pyrealsense2 as rs
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float64
from cv_bridge import CvBridge
import cv2
import apriltag
import numpy as np
from scipy.signal import butter, filtfilt

class AprilTagDepthDetector:
    def __init__(self):
        rospy.init_node('apriltag_depth_detector', anonymous=True)
        
        # RealSense pipeline setup
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        
        # Start streaming
        self.pipeline.start(self.config)
        
        self.bridge = CvBridge()
        # self.image_pub = rospy.Publisher('/magmed_stereoCamera_D405/image', Image, queue_size=10)
        self.depth_pub = rospy.Publisher('/magmed_stereoCamera_D405/april_tag_depth', Float64, queue_size=10)
        
        self.detector = apriltag.Detector()

        # Low-pass filter parameters
        self.fs = 15  # Sampling frequency (RealSense is streaming at 30 FPS)
        self.cutoff_freq = 0.2  # Cutoff frequency for the low-pass filter
        self.b, self.a = butter(2, self.cutoff_freq / (self.fs / 2), btype='low')
        
        # To store the last few depth measurements for filtering
        self.depth_measurements = []

        rospy.loginfo("AprilTag Depth Detector Initialized")
    
    def apply_low_pass_filter(self, depth):
        # Append the new depth measurement
        self.depth_measurements.append(depth)
        
        # Limit the list to the last N measurements (choose N based on filter order and requirements)
        if len(self.depth_measurements) > 50:  # Here 10 is arbitrary; you can adjust based on performance needs
            self.depth_measurements.pop(0)
        
        # Apply the filter if we have enough data points
        if len(self.depth_measurements) > 50:
            filtered_depth = filtfilt(self.b, self.a, self.depth_measurements)
            return filtered_depth[-1]  # Return the most recent filtered value
        else:
            return depth  # If not enough data, return the original depth
    
    def run(self):
        while not rospy.is_shutdown():
            frames = self.pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            
            if not depth_frame or not color_frame:
                continue
            
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            
            gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
            detections = self.detector.detect(gray_image)
            
            for detection in detections:
                (ptA, ptB, ptC, ptD) = detection.corners
                ptA = (int(ptA[0]), int(ptA[1]))
                ptB = (int(ptB[0]), int(ptB[1]))
                ptC = (int(ptC[0]), int(ptC[1]))
                ptD = (int(ptD[0]), int(ptD[1]))
                
                cv2.line(color_image, ptA, ptB, (0, 255, 0), 2)
                cv2.line(color_image, ptB, ptC, (0, 255, 0), 2)
                cv2.line(color_image, ptC, ptD, (0, 255, 0), 2)
                cv2.line(color_image, ptD, ptA, (0, 255, 0), 2)
                
                (cX, cY) = (int(detection.center[0]), int(detection.center[1]))
                depth = depth_frame.get_distance(cX, cY)

                # self.depth_pub.publish(depth)
                # Apply low-pass filter to the depth data
                filtered_depth = self.apply_low_pass_filter(depth)
                
                cv2.circle(color_image, (cX, cY), 5, (0, 0, 255), -1)
                cv2.putText(color_image, f"Depth: {filtered_depth:.2f}m", (cX, cY - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                
                self.depth_pub.publish(filtered_depth)

                # # Print filtered depth
                # rospy.loginfo(f"Filtered Depth: {filtered_depth:.5f}m")
            
            # image_message = self.bridge.cv2_to_imgmsg(color_image, "bgr8")
            # self.image_pub.publish(image_message)
            
            # cv2.imshow('AprilTag Depth Detector', color_image)
            # cv2.waitKey(1)
        
        self.pipeline.stop()

if __name__ == '__main__':
    try:
        detector = AprilTagDepthDetector()
        detector.run()
    except rospy.ROSInterruptException:
        pass
    cv2.destroyAllWindows()
