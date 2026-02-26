import time
from L1_lidar import Lidar
import L2_vector

LIDAR_IP = "192.168.6.172"

lidarsensor = Lidar(IP=LIDAR_IP)
lidarsensor.connect()
processor = lidarsensor.run()
time.sleep(1)

try:
    while True:
        scan = lidarsensor.get()
        if scan is None:
            time.sleep(0.05)
            continue

        r, ang = L2_vector.getNearest(scan)
        x, y = L2_vector.polar2cart(float(r), float(ang))

        # Write distance
        with open("/tmp/distance.txt", "w") as f:
            f.write(str(float(r)))

        # Write angle
        with open("/tmp/angle.txt", "w") as f:
            f.write(str(float(ang)))

        # Write x
        with open("/tmp/x.txt", "w") as f:
            f.write(str(float(x)))

        # Write y
        with open("/tmp/y.txt", "w") as f:
            f.write(str(float(y)))

        time.sleep(0.1)

except KeyboardInterrupt:
    pass
finally:
    lidarsensor.kill(processor)
