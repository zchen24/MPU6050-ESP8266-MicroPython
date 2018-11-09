# MPU6050 Library for MicroPython

from machine import I2C


class accel():
    def __init__(self, i2c: I2C, addr=0x68):
        self.iic = i2c
        self.addr = addr
        self.iic.start()
        self.iic.writeto(self.addr, bytearray([107, 0]))
        self.iic.stop()

    def get_raw_values(self):
        self.iic.start()
        a = self.iic.readfrom_mem(self.addr, 0x3B, 14)
        self.iic.stop()
        return a

    def get_ints(self):
        b = self.get_raw_values()
        c = []
        for i in b:
            c.append(i)
        return c

    def bytes_toint(self, firstbyte, secondbyte):
        if not firstbyte & 0x80:
            return firstbyte << 8 | secondbyte
        return - (((firstbyte ^ 255) << 8) | (secondbyte ^ 255) + 1)

    def get_values(self):
        raw_ints = self.get_raw_values()
        vals = {}
        vals["AcX"] = self.bytes_toint(raw_ints[0], raw_ints[1])
        vals["AcY"] = self.bytes_toint(raw_ints[2], raw_ints[3])
        vals["AcZ"] = self.bytes_toint(raw_ints[4], raw_ints[5])
        vals["Tmp"] = self.bytes_toint(raw_ints[6], raw_ints[7]) / 340.00 + 36.53
        vals["GyX"] = self.bytes_toint(raw_ints[8], raw_ints[9])
        vals["GyY"] = self.bytes_toint(raw_ints[10], raw_ints[11])
        vals["GyZ"] = self.bytes_toint(raw_ints[12], raw_ints[13])
        return vals  # returned in range of Int16
        # -32768 to 32767

    def set_gyro_range(self, FS_SEL=0):
        """ Set gyro full scale range. Ignore if FS_SEL is out of range.
        Args:
            FS_SEL (int): Full scale select.
                0: +/-  250 deg/s
                1: +/-  500 deg/s
                2: +/- 1000 deg/s
                3: +/- 2000 deg/s
        """
        if FS_SEL < 0 or FS_SEL > 3:
            return
        else:
            self.iic.writeto_mem(self.addr, 0x1B, FS_SEL << 3)

    def set_accel_range(self, AFS_SEL=0):
        """ Set accelerometer full scale range. Ignore if AFS_SEL is out of range.
        Args:
            AFS_SEL (int): Full scale select.
                0: +/- 2g
                1: +/- 4g
                2: +/- 8g
                3: +/- 16g
        """
        if AFS_SEL < 0 or AFS_SEL > 3:
            return
        else:
            self.iic.writeto_mem(self.addr, 0x1C, AFS_SEL << 3)

    def val_test(self):  # ONLY FOR TESTING! Also, fast reading sometimes crashes IIC
        from time import sleep
        while 1:
            print(self.get_values())
            sleep(0.05)
