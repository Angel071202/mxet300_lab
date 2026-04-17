# Project_Ball_Chase.py
# Detects a blue or pink ping pong ball and drives toward it.
# Blue is prioritized. Falls back to pink if blue not found.
# Uses a circularity filter to confirm the detected blob is a ball.
# Stops when the ball is close (TARGET_WIDTH pixels wide in frame).

import cv2
import numpy as np
import L2_speed_control as sc
import L2_inverse_kinematics as ik
import L2_kinematics as kin
import netifaces as ni
from time import sleep

# -----------------------------
# CAMERA SETUP
# -----------------------------
def getIp():
    for interface in ni.interfaces()[1:]:
        try:
            ip = ni.ifaddresses(interface)[ni.AF_INET][0]['addr']
            return ip
        except KeyError:
            continue
    return 0

stream_ip = getIp()
camera_input = 'http://' + stream_ip + ':8090/?action=stream' if stream_ip else 0

size_w = 240
size_h = 160
fov = 1.0   # camera field of view in radians (estimate)

# -----------------------------
# CALIBRATED HSV VALUES
# -----------------------------
# Blue (priority)
BLUE_H_MIN, BLUE_S_MIN, BLUE_V_MIN =  55,  95, 210
BLUE_H_MAX, BLUE_S_MAX, BLUE_V_MAX = 110, 165, 255

# Pink (fallback)
PINK_H_MIN, PINK_S_MIN, PINK_V_MIN =   0,  55, 250
PINK_H_MAX, PINK_S_MAX, PINK_V_MAX = 110, 165, 255

# -----------------------------
# CIRCULARITY FILTER SETTINGS
# -----------------------------
MIN_AREA        = 120   # ignore tiny blobs
MIN_CIRCULARITY = 0.70  # 1.0 = perfect circle
ASPECT_RATIO_TOL = 0.35 # w/h must be within this of 1.0

# -----------------------------
# BEHAVIOR SETTINGS
# -----------------------------
TARGET_WIDTH  = 70    # px — stop when ball is this wide (adjust to change stop distance)
WIDTH_MARGIN  = 8     # px — arrival tolerance on width
ANGLE_MARGIN  = 0.12  # rad — considered centered left/right
TURN_GAIN     = 1.1   # scales angular correction
FWD_GAIN      = 0.8   # scales forward drive effort

# -----------------------------
# BALL DETECTION
# -----------------------------
def find_ball(mask):
    """Return bounding rect (x,y,w,h) of best circular blob in mask, or None."""
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    best = None
    best_score = 0

    for c in cnts:
        area = cv2.contourArea(c)
        if area < MIN_AREA:
            continue

        perimeter = cv2.arcLength(c, True)
        if perimeter == 0:
            continue

        x, y, w, h = cv2.boundingRect(c)

        aspect = w / float(h)
        if abs(aspect - 1.0) > ASPECT_RATIO_TOL:
            continue

        circularity = 4.0 * np.pi * area / (perimeter * perimeter)
        if circularity < MIN_CIRCULARITY:
            continue

        score = area * circularity  # prefer large, round blobs
        if score > best_score:
            best_score = score
            best = (x, y, w, h)

    return best

# -----------------------------
# MAIN LOOP
# -----------------------------
def main():
    camera = cv2.VideoCapture(0)
    if not camera.isOpened():
        camera = cv2.VideoCapture(camera_input)
    if not camera.isOpened():
        print("Failed to open camera.")
        return

    camera.set(3, size_w)
    camera.set(4, size_h)

    print("Running. Ctrl+C to stop.")

    kernel = np.ones((5, 5), np.uint8)

    try:
        while True:
            sleep(0.05)

            ret, image = camera.read()
            if not ret:
                print("Failed to retrieve image!")
                break

            image = cv2.resize(image, (size_w, size_h))
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            _, width, _ = hsv.shape

            # --- Blue first ---
            mask_blue = cv2.inRange(hsv,
                (BLUE_H_MIN, BLUE_S_MIN, BLUE_V_MIN),
                (BLUE_H_MAX, BLUE_S_MAX, BLUE_V_MAX))
            mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, kernel)
            mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_CLOSE, kernel)

            ball = find_ball(mask_blue)
            color_name = "BLUE"

            # --- Fall back to pink ---
            if ball is None:
                mask_pink = cv2.inRange(hsv,
                    (PINK_H_MIN, PINK_S_MIN, PINK_V_MIN),
                    (PINK_H_MAX, PINK_S_MAX, PINK_V_MAX))
                mask_pink = cv2.morphologyEx(mask_pink, cv2.MORPH_OPEN, kernel)
                mask_pink = cv2.morphologyEx(mask_pink, cv2.MORPH_CLOSE, kernel)
                ball = find_ball(mask_pink)
                color_name = "PINK"

            # --- No ball ---
            if ball is None:
                print("No ball detected. Stopping.")
                sc.driveOpenLoop(np.array([0.0, 0.0]))
                continue

            x, y, w, h = ball
            center_x = x + w / 2.0
            angle = ((center_x / width) - 0.5) * fov   # negative = ball is left
            e_width = TARGET_WIDTH - w                  # positive = need to move closer

            wheel_measured = kin.getPdCurrent()

            # --- Arrived ---
            if abs(angle) < ANGLE_MARGIN and abs(e_width) < WIDTH_MARGIN:
                sc.driveOpenLoop(np.array([0.0, 0.0]))
                print(f"{color_name} ball reached. Stopped.")
                continue

            # --- Turn only until roughly centered, then add forward ---
            if abs(angle) < ANGLE_MARGIN:
                fwd  = FWD_GAIN * (e_width / TARGET_WIDTH)
            else:
                fwd = 0.0

            turn = -TURN_GAIN * angle
            wheel_speed = ik.getPdTargets(np.array([fwd, turn]))
            sc.driveClosedLoop(wheel_speed, wheel_measured, 0)

            print(f"{color_name} | angle={round(angle,3):+.3f} rad"
                  f" | width={w}px (target {TARGET_WIDTH}px)"
                  f" | wheels L={round(wheel_speed[0],2)} R={round(wheel_speed[1],2)}")

    except KeyboardInterrupt:
        pass

    finally:
        sc.driveOpenLoop(np.array([0.0, 0.0]))
        camera.release()
        print("Exiting.")

if __name__ == '__main__':
    main()
