#!/usr/bin/python

import smbus
import math

class MPU:

    # Power management registers
    power_mgmt_1 = 0x6b
    power_mgmt_2 = 0x6c

    ACCEL_XOUT_H = 0x3b
    ACCEL_XOUT_L = 0x3c

    ACCEL_YOUT_H = 0x3d
    ACCEL_YOUT_L = 0x3e

    ACCEL_ZOUT_H = 0x3f
    ACCEL_ZOUT_L = 0x40

    TEMP_OUT_H = 0x41
    TEMP_OUT_L = 0x42

    GYRO_XOUT_H = 0x43
    GYRO_XOUT_L = 0x44

    GYRO_YOUT_H = 0x45
    GYRO_YOUT_L = 0x46

    GYRO_ZOUT_H = 0x47
    GYRO_ZOUT_L = 0x48

    def __init__(self, address, bus_number):
        self.address = address
        self.bus_number = bus_number

        self.bus = smbus.SMBus(self.bus_number)
        #wake up device as it starts in sleep mode
        self.bus.write_byte_data(self.address, MPU.power_mgmt_1, 0)

    def read_byte(self, adr):
        return self.bus.read_byte_data(self.address, adr)

    def read_word(self, adr):
        high = self.bus.read_byte_data(self.address, adr)
        low = self.bus.read_byte_data(self.address, adr+1)
        val = (high << 8) + low
        return val

    def read_word_2c(self, adr):
        val = self.read_word(adr)
        if (val >= 0x8000):
            return -((65535 - val) + 1)
        else:
            return val

    def dist(self, a, b):
        return math.sqrt((a*a)+(b*b))

    def get_y_rotation(self, x, y, z):
        radians = math.atan2(x, self.dist(y,z))
        return -math.degrees(radians)

    def get_x_rotation(self, x, y, z):
        radians = math.atan2(y, self.dist(x,z))
        return math.degrees(radians)

    def get_temp_celsius(self):
        temp = self.read_word_2c(MPU.TEMP_OUT_H)
        return temp / 340.0 + 36.53

    def get_gyro_data(self):
        #print "gyro data"
        #print "---------"

        gyro_xout = self.read_word_2c(MPU.GYRO_XOUT_H)
        gyro_yout = self.read_word_2c(MPU.GYRO_YOUT_H)
        gyro_zout = self.read_word_2c(MPU.GYRO_ZOUT_H)

        gyro_xout_scaled = gyro_xout / 131
        gyro_yout_scaled = gyro_yout / 131
        gyro_zout_scaled = gyro_zout / 131

        #print "gyro_xout: ", gyro_xout, " scaled: ", (gyro_xout / 131)
        #print "gyro_yout: ", gyro_yout, " scaled: ", (gyro_yout / 131)
        #print "gyro_zout: ", gyro_zout, " scaled: ", (gyro_zout / 131)

    def get_accelerometer_data(self):
        #print
        #print "accelerometer data"
        #print "------------------"

        accel_xout = self.read_word_2c(MPU.ACCEL_XOUT_H)
        accel_yout = self.read_word_2c(MPU.ACCEL_YOUT_H)
        accel_zout = self.read_word_2c(MPU.ACCEL_ZOUT_H)

        accel_xout_scaled = accel_xout / 16384.0
        accel_yout_scaled = accel_yout / 16384.0
        accel_zout_scaled = accel_zout / 16384.0

        #print "accel_xout: ", accel_xout, " scaled: ", accel_xout_scaled
        #print "accel_yout: ", accel_yout, " scaled: ", accel_yout_scaled
        #print "accel_zout: ", accel_zout, " scaled: ", accel_zout_scaled

        x_rotation =  self.get_x_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled)
        y_rotation =  self.get_y_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled)

        #print "x rotation: ", x_rotation
        #print "y rotation: ", y_rotation

        return [x_rotation, y_rotation]
