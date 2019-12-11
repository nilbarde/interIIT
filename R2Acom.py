import serial
import time

ser = serial.Serial('/dev/ttyACM1', 9600)
a = 0

while True:
    a += 1
    x = (str(a)+"\n").encode()
    ser.write(x)
    print(a,x)
    time.sleep(1)
