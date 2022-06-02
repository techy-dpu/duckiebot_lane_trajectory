#!/usr/bin/env python3

import cv2 as cv
import numpy as np
import math

def edge_detection(frame):
    # Converts frame to grayscale because we only need the grey channel for detecting edges - less computationally expensive
    gray = cv.cvtColor(frame, cv.COLOR_RGB2GRAY)
    # Applies a 5x5 gaussian blur with deviation of 0 to frame - not mandatory since Canny will do this for us
    #cv.imshow("Noise reduced", gray)
    blur = cv.GaussianBlur(gray, (5, 5), 0)
    # Applies Canny edge detector with minVal of 150 and maxVal of 200
    # These values can be modified based on the image data
    canny = cv.Canny(blur, 150, 200)
    return canny

# This function crops the Region Of Interest from the frame as we want to ignore the unnecessary image pixels
def mask_ROI(frame):
    height, width = frame.shape
    #print (frame.shape)
    # Creates a rectangular polygon for the mask defined by three (x, y) coordinates
    # These values can be modified based on the image data
    polygons = np.array([
                            [ (0, int(height * 0.33)),(int(width * 0.85), int(height * 0.33)), (width, height), (0, height)]
                        ])
    # Creates an image filled with zero intensities with the same dimensions as the frame
    mask = np.zeros_like(frame)
    # Allows the mask to be filled with values of 1 and the other areas to be filled with values of 0
    cv.fillPoly(mask, polygons, 255)
    # A bitwise and operation between the mask and frame keeps only the triangular area of the frame
    roi = cv.bitwise_and(frame, mask)
    return roi

def average_lines(frame, lines):
    # Empty arrays to store the coordinates of the left and right lines
    left = []
    right = []
    # Loops through every detected line
    if lines is None:
       return np.array([[0,0,0,0], [0,0,0,0]])
    
    for line in lines:
        # Reshapes line from 2D array to 1D array
        x1, y1, x2, y2 = line.reshape(4)
        # Fits a linear polynomial to the x and y coordinates and returns a vector of coefficients which describe the slope and y-intercept
        parameters = np.polyfit((x1, x2), (y1, y2), 1)
        slope = parameters[0]
        y_intercept = parameters[1]
        # If slope is negative, the line is to the left of the lane, and otherwise, the line is to the right of the lane
        if slope < 0:
            left.append((slope, y_intercept))
        else:
            right.append((slope, y_intercept))
    # Averages out all the values for left and right into a single slope and y-intercept value for each line
    left_avg = np.average(left, axis = 0)
    right_avg = np.average(right, axis = 0)
    # Calculates the x1, y1, x2, y2 coordinates for the left and right lines
    left_line = get_line_coordinates(frame, left_avg)
    right_line = get_line_coordinates(frame, right_avg)
    return np.array([left_line, right_line])

def get_line_coordinates(frame, parameters):
    try:
        slope, intercept = parameters
    except TypeError:
        slope, intercept = 0,0 
    #slope, intercept = parameters
    # Sets initial y-coordinate as height from top down (bottom of the frame)
    y1 = frame.shape[0]
    # Sets final y-coordinate as 150 above the bottom of the frame
    y2 = int(y1 - y1*0.64)
    if slope != 0:
      # Sets initial x-coordinate as (y1 - b) / m since y1 = mx1 + b
      x1 = int((y1 - intercept) / slope)
      # Sets final x-coordinate as (y2 - b) / m since y2 = mx2 + b
      x2 = int((y2 - intercept) / slope)
    else:
      x1 = 0
      x2 = 0
    return np.array([x1, y1, x2, y2])

def replace_yellow_lines (image):
	hsv=cv.cvtColor(image,cv.COLOR_BGR2HSV)
	# Define lower and uppper limits of what we call "yellow"
	#yellow_lo=np.array([20,100,100])
	#yellow_hi=np.array([30,255,255])
	yellow_lo=np.array([20,90,90])
	yellow_hi=np.array([40,255,255])

	# Mask image to only select yellows
	mask=cv.inRange(hsv,yellow_lo,yellow_hi)

	# Change image to grey color what matches the road in the image
	image[mask>0]=(58,57,52)
	return image


def angle_of_line(x1, y1, x2, y2):
    parameters = np.polyfit((x1, x2), (y1, y2), 1)
    slope = parameters[0]
    y_intercept = parameters[1]
        
    myradians = math.degrees(math.atan2(-y1-y2, x2-x1))
    #mydegrees = math.degrees(myradians)
    #myradians = math.radians(mydegrees)
    return myradians, slope
    
def draw_lines_left_right (frame, lines):
    # Creates an image filled with zero intensities with the same dimensions as the frame
    lines_left = np.zeros_like(frame)
    lines_right = np.zeros_like(frame)
    lines_center = np.zeros_like(frame)
    # Checks if any lines are detected
    if lines is not None:
        x1, y1, x2, y2 =  lines[0]
        # Draws lines between two coordinates with green color and 5 thickness
        cv.line(lines_left, (x1, y1), (x2, y2), (0, 0, 255), 5)
        x1, y1, x2, y2 =  lines[1]
        # Draws lines between two coordinates with green color and 5 thickness
        cv.line(lines_right, (x1, y1), (x2, y2), (0, 0, 255), 5)

        x1, y1, x2, y2 = np.array((( lines[0] + lines[1] ) / 2)).astype(int)
        print (lines[0] , lines[1], x1, y1, x2, y2)
        # Draws lines between two coordinates with green color and 5 thickness
        cv.line(lines_center, (x1, y1), (x2, y2), (0, 0, 255), 5)
        
    return lines_left, lines_right, lines_center
    
    
def draw_lines(frame, lines):
    # Creates an image filled with zero intensities with the same dimensions as the frame
    lines_visualize = np.zeros_like(frame)
    # Checks if any lines are detected
    if lines is not None:
        for x1, y1, x2, y2 in lines:
            # Draws lines between two coordinates with green color and 5 thickness
            cv.line(lines_visualize, (x1, y1), (x2, y2), (0, 0, 255), 5)
    return lines_visualize

def draw_lines_p (img, linesP):
    if linesP is not None:
        for i in range(0, len(linesP)):
            l = linesP[i][0]
            cv.line(img, (l[0], l[1]), (l[2], l[3]), (0,0,255), 3, cv.LINE_AA)
    return img
    
def detect_lanes (frame):
    # ret = a boolean return value from getting the frame, frame = the current frame being projected in the video
    #cv.imshow("Input Image", frame)
    canny = edge_detection(frame)
    #cv.imshow("canny", canny)
    # plt.imshow(frame)
    # plt.show()
    segment = mask_ROI(canny)
    hough = cv.HoughLinesP(segment, 2, np.pi / 180, 100, np.array([]), minLineLength = 100, maxLineGap = 20)
    # Averages multiple detected lines from hough into one line for left border of lane and one line for right border of lane
    #print (hough)
    lines = average_lines(frame, hough)
    # Visualizes the lines
    #lines_visualize = draw_lines(frame, lines)
    lt_mask, rt_mask, ct_mask = draw_lines_left_right (frame, lines)
    # center of left and right lines
    x1, y1, x2, y2 = np.array((( lines[0] + lines[1] ) / 2)).astype(int)
    #x1, y1, x2, y2 = lines[0]
    # angle of the center line
    angle, slope = angle_of_line(x1, y1, x2, y2)
    #cv.imshow("Lines ", lines_visualize)
    #cv.imshow("lt_mask ", lt_mask)
    #cv.imshow("rt_mask ", rt_mask)
    # Overlays lines on frame by taking their weighted sums and adding an arbitrary scalar value of 1 as the gamma argument
    #output = cv.addWeighted(frame, 0.9, lines_visualize, 1, 1)
    # Opens a new window and displays the output frame
    #draw_lines_p (frame, hough)
    #cv.imshow("detected lanes", frame)
    #cv.imshow("ROI segment", segment)
    #cv.imshow("output with lanes", output)
    # Frames are read by intervals of 10 milliseconds. The programs breaks out of the while loop when the user presses the 'q' key

    return lt_mask, rt_mask, angle, slope


