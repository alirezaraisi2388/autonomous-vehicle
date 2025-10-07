import math
from picamera2 import Picamera2
import cv2
import numpy as np

def detect_edges(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_white = np.array([0, 0, 200])
    upper_white = np.array([180, 30, 255])
    mask = cv2.inRange(hsv, lower_white, upper_white)
    edges = cv2.Canny(mask, 200, 400)
    return edges

def region_of_interest(edges):
    height, width = edges.shape
    mask = np.zeros_like(edges)
    polygon = np.array([[
        (0, height * 1/2),
        (width, height * 1/2),
        (width, height),
        (0, height),
    ]], np.int32)
    cv2.fillPoly(mask, polygon, 255)
    cropped_edges = cv2.bitwise_and(edges, mask)
    return cropped_edges

def detect_line_segments(cropped_edges):
    rho = 1
    angle = np.pi / 180
    min_threshold = 10
    return cv2.HoughLinesP(cropped_edges, rho, angle, min_threshold, 
                           np.array([]), minLineLength=8, maxLineGap=4)

def average_slope_intercept(frame, line_segments):
    lane_lines = []
    if line_segments is None:
        return lane_lines
    height, width, _ = frame.shape
    left_fit = []
    right_fit = []
    boundary = 1/3
    left_region_boundary = width * (1 - boundary)
    right_region_boundary = width * boundary

    for line_segment in line_segments:
        for x1, y1, x2, y2 in line_segment:
            if x1 == x2:
                continue
            fit = np.polyfit((x1, x2), (y1, y2), 1)
            slope = fit[0]
            intercept = fit[1]
            if slope < 0:
                if x1 < left_region_boundary and x2 < left_region_boundary:
                    left_fit.append((slope, intercept))
            else:
                if x1 > right_region_boundary and x2 > right_region_boundary:
                    right_fit.append((slope, intercept))

    if len(left_fit) > 0:
        left_fit_average = np.average(left_fit, axis=0)
        lane_lines.append(make_points(frame, left_fit_average))
    if len(right_fit) > 0:
        right_fit_average = np.average(right_fit, axis=0)
        lane_lines.append(make_points(frame, right_fit_average))

    return lane_lines

def make_points(frame, line):
    height, width, _ = frame.shape
    slope, intercept = line
    y1 = height
    y2 = int(y1 * 1 / 2)
    x1 = max(-width, min(2 * width, int((y1 - intercept) / slope)))
    x2 = max(-width, min(2 * width, int((y2 - intercept) / slope)))
    return [[x1, y1, x2, y2]]

def display_lines(frame, lines, line_color=(0, 255, 0), line_width=2):
    line_image = np.zeros_like(frame)
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)
    line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
    return line_image

def detect_lane(frame):
    edges = detect_edges(frame)
    cropped_edges = region_of_interest(edges)
    line_segments = detect_line_segments(cropped_edges)
    lane_lines = average_slope_intercept(frame, line_segments)
    return lane_lines

def stabilize_steering_angle(curr_angle, new_angle, num_lanes):
    max_angle_deviation = 5 if num_lanes == 2 else 10
    angle_deviation = new_angle - curr_angle
    if abs(angle_deviation) > max_angle_deviation:
        stabilized_angle = curr_angle + max_angle_deviation * np.sign(angle_deviation)
    else:
        stabilized_angle = new_angle
    return stabilized_angle

def display_heading_line(frame, steering_angle, line_color=(0, 0, 255), line_width=5):
    heading_image = np.zeros_like(frame)
    height, width, _ = frame.shape
    steering_angle_radian = steering_angle / 180.0 * np.pi
    x1 = int(width / 2)
    y1 = height
    x2 = int(x1 - height / 2 / np.tan(steering_angle_radian))
    y2 = int(height / 2)
    cv2.line(heading_image, (x1, y1), (x2, y2), line_color, line_width)
    return cv2.addWeighted(frame, 0.8, heading_image, 1, 1)


# code pishnahadi chatgpt baray mohasebye 
def get_steering_angle(frame):
    lane_lines = detect_lane(frame)
    if lane_lines:
        _, _, left_x2, _ = lane_lines[0][0]
        if len(lane_lines) == 2:
            _, _, right_x2, _ = lane_lines[1][0]
            x_offset = (left_x2 + right_x2) / 2 - (frame.shape[1] / 2)
        else:
            x1, _, x2, _ = lane_lines[0][0]
            x_offset = x2 - x1

        y_offset = int(frame.shape[0] / 2)
        angle_to_mid_radian = math.atan(x_offset / y_offset)
        angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)
        return angle_to_mid_deg + 90
    return 90  # zavieye pishfarz


