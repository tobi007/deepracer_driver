
#!/usr/bin/env python
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

import cv2
import numpy as np
import matplotlib
import logging
import math

from keras.models import load_model

import argparse

class Driver(Node):
    def __init__(self, params):
        super().__init__('cv_runner')
        self.subscription = self.create_subscription(
            Image,
            '/zed_camera_left_sensor/image_raw',
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.curr_steering_angle = 0
        
        self.publisher_ = self.create_publisher(Twist, '/racer_car/cmd_vel', 1)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.compute_cmd_vel)
        self.i = 10032

        self.runner_dir = params['runner_dir']
        self.use_open_cv = params['use_opvencv']
        if self.use_open_cv:
            self.curr_throttle_spd = 0.5
        else:
            self.model = load_model('%s/note-book/lane_navigation_check_87.17.h5' % self.runner_dir)
            self.curr_throttle_spd = 0.5
    
    def compute_cmd_vel(self):
        # 86-95 heading straignt  45–85 degrees is turning left, and 96–135 degrees is turning right
        # linear x (forward +ve backward -ve) angular z (left +ve right -ve)
        msg = Twist()
        

        if self.curr_steering_angle > 0 and self.curr_steering_angle <= 85:
            msg.angular.z = 1.0
            msg.linear.x = self.curr_throttle_spd
        elif self.curr_steering_angle > 85 and self.curr_steering_angle <= 95:
            msg.angular.z = 0.0
            msg.linear.x = self.curr_throttle_spd
        elif self.curr_steering_angle > 96 and self.curr_steering_angle <= 180:
            msg.angular.z = -1.0
            msg.linear.x = self.curr_throttle_spd
        else:
            msg.linear.x = self.curr_throttle_spd
            msg.angular.z = 0.0

        self.publisher_.publish(msg)
        print('Turning Angle: "%s" degrees' % self.curr_steering_angle)
        

    def listener_callback(self, rc_image_msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(rc_image_msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        if self.use_open_cv:
            steering_angle, side_line_num = self.use_opencv_to_get_steering(cv_image)
        else:
            
            image = img_preprocess(cv_image)
            
            steering_angle = int(self.model.predict(np.asarray([image])))  
            side_line_num = 1
            print("Predicted Steer Angle: %03d" % (steering_angle))     
        
        # print("Computed Steering Angle: %s", steering_angle)
        # self.save_image_and_steering_angle(cv_image, steering_angle)
        # self.i += 1

        self.stabilize_steering_angle(steering_angle, side_line_num)
        side_lane_lines_heading_image = self.display_heading_line(cv_image, steering_angle)
        cv2.imshow("heeding side lane lines", side_lane_lines_heading_image)

        cv2.waitKey(3)
    
    def use_opencv_to_get_steering(self, frame):
        # Middle Line
        middle_line_lower_yellow = np.array([20, 40, 40])
        middle_line_upper_yellow = np.array([50, 255, 255])
        

        side_line_lower_white = np.array([-11, -10, 205])
        side_line_upper_white = np.array([ 59, 30, 305])
        

        # middle_line_edges = self.detect_edges(cv_image, middle_line_lower_yellow, middle_line_upper_yellow)
        side_line_edges = self.detect_edges(frame, side_line_lower_white, side_line_upper_white)
        

        # middle_line_roi = self.region_of_interest(middle_line_edges)
        side_line_roi = self.region_of_interest(side_line_edges)
        # cv2.imshow('MIDDLE LINE ROI', middle_line_roi)
        cv2.imshow('SIDE LINE ROI', side_line_roi)

        # middle_line_segments = self.detect_line_segments(middle_line_roi)
        side_line_segments = self.detect_line_segments(side_line_roi)

        # # middle_lines_slope = self.average_slope_intercept(cv_image, middle_line_segments)
        side_lines_slope = self.average_slope_intercept(frame, side_line_segments)

        # # middle_lane_lines_image = self.display_lines(cv_image, middle_lines_slope)
        # # cv2.imshow("middle lane lines", middle_lane_lines_image)

        side_lane_lines_image = self.display_lines(frame, side_lines_slope)
        cv2.imshow("side lane lines", side_lane_lines_image)

        steering_angle = self.compute_steering_angle(frame, side_lines_slope)

        return steering_angle, len(side_lines_slope)
    
    def save_image_and_steering_angle(self, frame, steering_angle):
        image_folder = "%s/image_data/" %  self.runner_dir
        print("%s%03d_%03d.png" % (image_folder, self.i, steering_angle))
        cv2.imwrite("%s%03d_%03d.png" % (image_folder, self.i, steering_angle), frame)

    def detect_edges(self, frame, lower_blue, upper_blue):
        # filter for blue lane lines
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
    
        # Dilate and erode to reduce the lane thickness
        # height, width = mask.shape
        # skel = np.zeros([height, width], dtype=np.uint8)
        # kernel = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
        # temp_nonzero = np.count_nonzero(mask)
        # while(np.count_nonzero(mask) != 0):
        #     eroded = cv2.erode(mask, kernel)
        #     temp = cv2.dilate(eroded, kernel)

        #     temp = cv2.subtract(mask, temp)
        #     skel = cv2.bitwise_or(skel, temp)
        #     mask = eroded.copy()
        

        # detect edges
        edges = cv2.Canny(mask, 200, 400)

        return edges

    def region_of_interest(self, edges):
        height, width = edges.shape
        mask = np.zeros_like(edges)

        # only focus bottom half of the screen
        polygon = np.array([[
            (0, height * 1 / 3),
            (width, height * 1 / 3),
            (width, height),
            (0, height),
        ]], np.int32)

        cv2.fillPoly(mask, polygon, 255)
        cropped_edges = cv2.bitwise_and(edges, mask)

        return cropped_edges

    def detect_line_segments(self, cropped_edges):
        # tuning min_threshold, minLineLength, maxLineGap is a trial and error process by hand
        rho = 1  # distance precision in pixel, i.e. 1 pixel
        angle = np.pi / 180  # angular precision in radian, i.e. 1 degree
        min_threshold = 10  # minimal of votes
        line_segments = cv2.HoughLinesP(cropped_edges, rho, angle, min_threshold, 
                                        np.array([]), minLineLength=10, maxLineGap=4)

        # if line_segments is not None:
        #     print('detected line_segment >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>:')
        #     for line_segment in line_segments:
        #         print("%s of length %s" % (line_segment, length_of_line_segment(line_segment[0])))

        return line_segments

    def average_slope_intercept(self, frame, line_segments):
        """
        This function combines line segments into one or two lane lines
        If all line slopes are < 0: then we only have detected left lane
        If all line slopes are > 0: then we only have detected right lane
        """
        lane_lines = []
        if line_segments is None:
            print('No line_segment segments detected')
            return lane_lines

        height, width, _ = frame.shape
        left_fit = []
        right_fit = []
        left_fit_length = 0
        right_fit_length = 0

        boundary = 1/3
        left_region_boundary = width * (1 - boundary)  # left lane line segment should be on left 2/3 of the screen
        right_region_boundary = width * boundary # right lane line segment should be on left 2/3 of the screen


        for line_segment in line_segments:
            for x1, y1, x2, y2 in line_segment:
                if x1 == x2:
                    print('skipping vertical line segment (slope=inf): %s' % line_segment)
                    continue
                if abs(y1 - y2) < 3:
                    print('skipping horizontal line segment (slope=inf): %s' % line_segment)
                    continue
                fit = np.polyfit((x1, x2), (y1, y2), 1)
                slope = fit[0]
                intercept = fit[1]
                if slope < 0:
                    if x1 < left_region_boundary and x2 < left_region_boundary and length_of_line_segment(line_segment[0]) > left_fit_length:
                        left_fit = make_points(frame, (slope, intercept)) #line_segment
                        left_fit_length = length_of_line_segment(line_segment[0])
                        print("Left Slope/Intercept: ", line_segment)
                else:
                    if x1 > right_region_boundary and x2 > right_region_boundary and length_of_line_segment(line_segment[0]) > right_fit_length:
                        right_fit = make_points(frame, (slope, intercept)) #line_segment
                        right_fit_length = length_of_line_segment(line_segment[0])
                        print("Right Slope/Intercept: ", line_segment)

        if len(left_fit) > 0:
            lane_lines.append(left_fit)

        if len(right_fit) > 0:
            lane_lines.append(right_fit)

        #print('lane lines: %s' % lane_lines)  # [[[316, 720, 484, 432]], [[1009, 720, 718, 432]]]

        return lane_lines

    def display_lines(self, frame, lines, line_color=(0, 255, 0), line_width=10):
        line_image = np.zeros_like(frame)
        if lines is not None:
            for line in lines:
                for x1, y1, x2, y2 in line:
                    cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)
        line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
        return line_image

    def display_heading_line(self, frame, steering_angle, line_color=(0, 0, 255), line_width=5 ):
        heading_image = np.zeros_like(frame)
        height, width, _ = frame.shape

        # figure out the heading line from steering angle
        # heading line (x1,y1) is always center bottom of the screen
        # (x2, y2) requires a bit of trigonometry

        # Note: the steering angle of:
        # 0-89 degree: turn left
        # 90 degree: going straight
        # 91-180 degree: turn right 
        steering_angle_radian = steering_angle / 180.0 * math.pi
        x1 = int(width / 2)
        y1 = height
        x2 = int(x1 - height / 2 / math.tan(steering_angle_radian))
        y2 = int(height / 2)

        cv2.line(heading_image, (x1, y1), (x2, y2), line_color, line_width)
        heading_image = cv2.addWeighted(frame, 0.8, heading_image, 1, 1)

        return heading_image

    def compute_steering_angle(self, frame, lane_lines):
        """ Find the steering angle based on lane line coordinate
        We assume that camera is calibrated to point to dead center
        """
        if len(lane_lines) == 0:
            print('No lane lines detected, do nothing')
            return -90

        height, width, _ = frame.shape
        
        if len(lane_lines) == 1:
            #print('Only detected one lane line, just follow it. %s' % lane_lines[0])
            x1, _, x2, _ = lane_lines[0][0]
            x_offset = x2 - x1
        else:
            _, _, left_x2, _ = lane_lines[0][0]
            _, _, right_x2, _ = lane_lines[1][0]
            camera_mid_offset_percent = 0.02 # 0.0 means car pointing to center, -0.03: car is centered to left, +0.03 means car pointing to right
            mid = int(width / 2 * (1 + camera_mid_offset_percent))
            x_offset = (left_x2 + right_x2) / 2 - mid

        # find the steering angle, which is angle between navigation direction to end of center line
        y_offset = int(height / 2)

        angle_to_mid_radian = math.atan(x_offset / y_offset)  # angle (in radian) to center vertical line
        angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)  # angle (in degrees) to center vertical line
        steering_angle = 180 - (angle_to_mid_deg + 90)  # this is the steering angle needed by picar front wheel

        #print('new steering angle: %s' % steering_angle)
        return steering_angle

    def stabilize_steering_angle(self, new_steering_angle, num_of_lane_lines, max_angle_deviation_two_lines=10, max_angle_deviation_one_lane=30):
        """
        Using last steering angle to stabilize the steering angle
        This can be improved to use last N angles, etc
        if new angle is too different from current angle, only turn by max_angle_deviation degrees
        """
        if num_of_lane_lines == 2 :
            # if both lane lines detected, then we can deviate more
            max_angle_deviation = max_angle_deviation_two_lines
        else :
            # if only one lane detected, don't deviate too much
            max_angle_deviation = max_angle_deviation_one_lane
        
        angle_deviation = new_steering_angle - self.curr_steering_angle
        if abs(angle_deviation) > max_angle_deviation:
            self.curr_steering_angle = int(self.curr_steering_angle
                                            + max_angle_deviation * angle_deviation / abs(angle_deviation))
        else:
            self.curr_steering_angle = new_steering_angle
        #print('Proposed angle: %s, stabilized angle: %s' % (new_steering_angle, self.curr_steering_angle))
        




############################
# Utility Functions
############################

def img_preprocess(image):
    height, _, _ = image.shape
    image = image[int(height/3):,:,:]  # remove top half of the image, as it is not relavant for lane following
    image = cv2.cvtColor(image, cv2.COLOR_RGB2YUV)  # Nvidia model said it is best to use YUV color space
    image = cv2.GaussianBlur(image, (3,3), 0)
    image = cv2.resize(image, (200,66)) # input image size (200,66) Nvidia model
    image = image / 255 # normalizing, the processed image becomes black for some reason.  do we need this?
    return image


def length_of_line_segment(line):
    x1, y1, x2, y2 = line
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def make_points(frame, line):
    height, width, _ = frame.shape
    slope, intercept = line
    
    # y1 = height  # bottom of the frame
    # y2 = int(y1 * 1 / 2)  # make points from middle of the frame down

    # # bound the coordinates within the frame
    # x1 = max(-width, min(2 * width, int((y1 - intercept) / slope))) 
    # x2 = max(-width, min(2 * width, int((y2 - intercept) / slope)))
    # return [[x1, y1, x2, y2]]
    x2 = 0
    x1 = int(width / 2)
    if slope > 0:
        x2 = width
        x1 = int(width * 2 / 3)
    y2 = int(x2 * slope + intercept)
    y1 = int(x1 * slope + intercept)
    return [[x1, y1, x2, y2]]
    

    

    
def main():
    rclpy.init()


    text = 'This is a test program. It demonstrates how to use the argparse module with a program description.'

    # initiate the parser with a description
    parser = argparse.ArgumentParser(description = text)
    

    parser.add_argument("--use_deeplearnig", 
                        help="If True, use deep learning trained model to compute steering angle",
                        default=False, 
                        action="store_true"
                        )

    parser.add_argument("--runner_dir", 
                        help="Sets the absolute directory for the deepracer_runner package",
                        type=str,
                        default=""
                        )

    args = parser.parse_args()
    
    config = {
        "use_opvencv": not args.use_deeplearnig,
        "runner_dir": args.runner_dir,
    }

    driver = Driver(config)

    rclpy.spin(driver)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    runner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()