import json
import time
from serial import Serial
import serial
import os
import serial.tools.list_ports
from class_robot import Robot

class MCP4786:
    def __init__(self):
        ports = list(serial.tools.list_ports.comports())
        for port in ports:
            print(str(port).split(' ')[2])
        print("echo 1212 | sudo -S chmod 666 /dev/" + str(port).split(' ')[2])
        os.system("echo 1212 | sudo -S chmod 666 /dev/" +
                  str(port).split(' ')[2])
        # 打开串口
        self.__ser = serial.Serial(timeout=1, port="/dev/ttyACM2",
                                   baudrate=115200, bytesize=8, parity="N", stopbits=1)
        if not self.__ser.isOpen():
            print('串口没打开')
            exit(0)
        self.mcp4786_write_voltage([0, 0, 0, 0])

    def mcp4786_write_voltage(self, voltage):
        # time.sleep(0.02)
        d1 = voltage[0]  # 0-5000 电压 0-5v
        d2 = voltage[1]
        d3 = voltage[2]
        d4 = voltage[3]

        new_parm = {"cmd": 2,
                    "d1": d1,
                    "d2": d2,
                    "d3": d3,
                    "d4": d4,
                    }
        cmd_json = json.dumps(new_parm) + '\n'
        self.__ser.write(cmd_json.encode('utf-8'))
            # print("发送数据：", new_parm)

    def mcp4786_close(self):
        # 关闭串口
        self.__ser.close()

# %%
if __name__ == '__main__':
    #%%
    mcp = MCP4786()
    time.sleep(2)
    # %%
    mcp.mcp4786_write_voltage([1200, 0, 1200, 0])
    time.sleep(2)
    # %%
    mcp.mcp4786_write_voltage([3300, 9999, 3800, 9999])
    time.sleep(2)
    # %%
    mcp.mcp4786_write_voltage([0, 0, 0, 0])
    time.sleep(2)
    #%%
    mcp.mcp4786_close()
