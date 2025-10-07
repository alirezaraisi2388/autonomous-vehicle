from MotorModule import Motor
from image_processing import detect_lane, get_steering_angle, stabilize_steering_angle, display_heading_line
import cv2
from picamera2 import Picamera2

motor = Motor(2, 3, 23, 22, 17, 27)
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (640, 480)})
picam2.configure(config)
picam2.start()

curr_steering_angle = 90  # Initial steering angle

def main():
    frame = picam2.capture_array()
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    # Get lane lines
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
        new_steering_angle = angle_to_mid_deg + 90

        stabilized_steering_angle = stabilize_steering_angle(
            curr_steering_angle, new_steering_angle, len(lane_lines))
        curr_steering_angle = stabilized_steering_angle

        frame = display_heading_line(frame, stabilized_steering_angle)

    # Display lane lines
    lane_lines_image = display_lines(frame, lane_lines)
    cv2.imshow("Lane Detection", lane_lines_image)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        return False
    return True

if __name__ == '__main__':
    try:
        while main():
            pass
    except KeyboardInterrupt:
        motor.stop()
    finally:
        picam2.stop()
        cv2.destroyAllWindows()
