import serial
from time import sleep
import sys

import serial
from time import sleep
import sys

COM_PORT = 'COM4'  # 請自行修改序列埠名稱
BAUD_RATES = 9600
ser = serial.Serial(COM_PORT, BAUD_RATES)

try:
    while True:
        # 接收用戶的輸入值並轉成小寫
        choice = input('TYPE: left_open, left_close, back_open, back_close, right_open, right_close ').lower()

        if choice == 'left_open':
            print('Sending command')
            ser.write(b'left_open\n') 
            sleep(0.5)             
        if choice == 'left_close':
            print('Sending command')
            ser.write(b'left_close\n') 
            sleep(0.5)             
        elif choice == 'back_open':
            print('Sending command')
            ser.write(b'back_open\n')
            sleep(0.5)
        elif choice == 'back_close':
            print('Sending command')
            ser.write(b'back_close\n')
            sleep(0.5)
        elif choice == 'right_open':
            print('Sending command')
            ser.write(b'right_open\n')
            sleep(0.5)
        elif choice == 'right_close':
            print('Sending command')
            ser.write(b'right_close\n')
            sleep(0.5)            
        elif choice == 'e':
            ser.close()
            print('再見！')
            sys.exit()


        while ser.in_waiting:
            mcu_feedback = ser.readline().decode()  
            print('Feedback：', mcu_feedback)
            
except KeyboardInterrupt:
    ser.close()
    print('再見！')