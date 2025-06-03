import serial
import time
from kinematics import inverse_kinematics

ser = serial.Serial('COM1', 9600)
time.sleep(2)
target_position = [0, 0, 0.5]

msg = ','.join(map(str, inverse_kinematics(target_position))) + '\n'
ser.write(msg.encode())