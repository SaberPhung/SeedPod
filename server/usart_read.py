import csv
import time
import serial
import datetime
from datetime import datetime

ser = serial.Serial(
        port='/dev/ttyACM0',
        baudrate = 115200,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=1
)
def sensor_data():
        timeC = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        temp = ser.readline()
        time.sleep(0.0001)
        data = [temp,timeC]
        print (data[1] + " "+  data[0])
        return (timeC + " "+ data[0].replace("\x00", ""))

f= open("testfinal.csv","a")
c= csv.writer(f)