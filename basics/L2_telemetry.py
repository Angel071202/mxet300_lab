import time
import L1_ina as ina      # uses readVolts() to get battery voltage
import L1_log as log      # uses tmpFile() to log to /tmp

FILE_NAME = "battery_voltage.txt"  

while True:
    v = ina.readVolts()      
    log.tmpFile(v, FILE_NAME)
    time.sleep(1)
    print('Voltage: ' + str(v) + "V")
