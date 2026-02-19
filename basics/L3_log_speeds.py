import time
import L2_kinematics as kin

while True:
    pdl, pdr = kin.getPdCurrent()
    xdot, thetadot = kin.getMotion()

    open("/tmp/pdl.txt", "w").write(str(float(pdl)))
    open("/tmp/pdr.txt", "w").write(str(float(pdr)))
    open("/tmp/xdot.txt", "w").write(str(float(xdot)))
    open("/tmp/thetadot.txt", "w").write(str(float(thetadot)))

    time.sleep(0.2)

