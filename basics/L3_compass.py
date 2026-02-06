import time
import numpy as np
import board
import adafruit_bno055
import L1_log

i2c = board.I2C()
imu = adafruit_bno055.BNO055_I2C(i2c)
imu.mode = adafruit_bno055.MAGONLY_MODE

x_min, x_max = 21.6875, 86.0
y_min, y_max = -34.1875, 26.0625
z_min, z_max = 80.9375, 146.375
declination_angle = 7

def calibrate_magnetometer(v, vmin, vmax):
    temp = (v - vmin) / (vmax - vmin)
    return 2 * temp - 1

def get_heading_deg():
    x, y, _ = imu.magnetic
    if x is None or y is None:
        return None
    x_c = calibrate_magnetometer(x, x_min, x_max)
    y_c = calibrate_magnetometer(y, y_min, y_max)
    h = np.degrees(np.arctan2(y_c, x_c)) - declination_angle
    h = -h
    while h > 180:
        h -= 360
    while h < -180:
        h += 360
    return h

def heading_to_cardinal(h):
    while h > 180:
        h -= 360
    while h < -180:
        h += 360

    if -19 <= h < 30:
        return "North"
    elif 30 <= h < 55:
        return "North East"
    elif 55 <= h < 105:
        return "East"
    elif 105 <= h < 130:
        return "South East"
    elif 130 <= h or h < -165:
        return "South"
    elif -165 <= h < -99:
        return "South West"
    elif -99 <= h < -30:
        return "West"
    elif -30 <= h < -19:
        return "North West"
    else:
        return "North"


def main():
    while True:
        h = get_heading_deg()
        if h is None:
            time.sleep(0.1)
            continue
        d = heading_to_cardinal(h)
        L1_log.tmpFile(h, "compassHeading.txt")
        L1_log.stringTmpFile(d, "compassDirection.txt")
        print(f"Heading: {h:.2f} deg, Direction: {d}")
        time.sleep(0.1)

if __name__ == "__main__":
    main()

