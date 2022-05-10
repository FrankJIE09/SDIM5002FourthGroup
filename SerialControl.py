import serial


class ComSwitch(object):
    def __init__(self, com="/COM3"):
        self.__open_cmd = [0xA0, 0x01, 0x01, 0xA2]
        self.__close_cmd = [0xA0, 0x01, 0x00, 0xA1]
        self.sc = serial.Serial(port=com, baudrate=9600)

    def open(self):
        self.sc.write(self.__open_cmd)

    def close(self):
        self.sc.write(self.__close_cmd)


if __name__ == "__main__":
    print('CH430 serial test!')
    import time

    cs = ComSwitch()
    cs.open()
    time.sleep(20)
    cs.close()
    time.sleep(0.5)
    # for i in range(5):
