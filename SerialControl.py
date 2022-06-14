import serial
import time


class ComSwitch(object):
    def __init__(self, com="/dev/ttyUSB0"):
        self.__open_cmd = 'C'.encode()
        self.__close_cmd = 'F'.encode()
        self.sc = serial.Serial(port=com, baudrate=115200)

    def open(self):
        self.sc.write(self.__open_cmd)

    def close(self):
        self.sc.write(self.__close_cmd)


if __name__ == "__main__":
    print('CH430 serial test!')
    cs = ComSwitch()
    cs.open()
    time.sleep(1)
    cs.close()
    time.sleep(1)
    # for i in range(5):
