# --coding=utf8--
import serial
import json
import time
import sys
import copy

# from jiont_test import trans_matrix, random_config

'''
# Servo 0,0 ~ 180, the initial position 90
# Servo 1, from right to left, 10 to 180, initial 95
# Servo 2, from top to bottom, 180 ~ 30, the initial 150
# Servo 3,0 ~ 180, initial 90
# Servo 4,0 ~ 90, initial 90
# Servo 5, 30 ~ 90, initial 90
# Change the parameter type to dict, remove the write_joint model parameter, only the absolute position
'''


class Control(object):

    def __init__(self, port="com6", baudR=9600):
        # make sure arduino and baud have same rate
        self.readFlag = "*"
        self.moveFlag = "#"
        self.resetFlag = "!"
        self.speed = 60
        self.receive_data = ""
        self.initPosition = [90, 95, 90, 90, 90, 90]
        self.direction = [1, -1, 1, 1, 1]
        self.curPosition = [90, 95, 90, 90, 90, 90]
        self.RelPosition = [90, 180, 180, 180, 90]
        self.initRelPosition = [90, 180, 180, 180, 90]
        self.min_ang = [0, 10, 30, 0, 0]
        self.max_ang = [180, 180, 180, 180, 90]
        try:
            self.ser = serial.Serial(port, baudR, timeout=1)
            self.ser.flush()
            time.sleep(2)
        except Exception, e:
            print Exception, ":", e
            exit()

    def global_reconnect(self):
        self.ser.flush()
        self.ser.close()
        time.sleep(2)
        self.ser.open()

    # Send serial information
    def send_data(self, command):
        if(self.ser.isOpen() == False):
            self.global_reconnect()
        self.ser.write(command)
        time.sleep(0.1)

    # Read the serial information
    def read_data(self):
        self.ser.flush()
        self.receive_data = self.ser.readline()
        return self.receive_data

    def read_joint(self, nums="*"):
        com = ""
        if nums == "*":
            com += self.readFlag + nums[0]
        else:
            for num in nums:
                number = self._toHex(str(num))
                com += self.readFlag + number
        print com
        self.send_data(com)
        angles = self.read_data()
        angle_list = angles.split(" ")
        angle_list.pop()
        print angle_list
        if len(angle_list):
            return angle_list
        else:
            print "Error,list is empty"
            return 0

    # Control the steering gear to move
    # model is the pattern, 0 is the absolute position, and 1 is the incremental position
    # dic is the steering gear and the corresponding angle
    def write_joint(self, l):
        l = self._position(l)
        if l is None:
            return
        com = self._get_command(l)
        if not com:
            return
        print com
        # self._get_rel_position(l)
        self.send_data(com)

    def _position(self, l):
        if len(l) > 5:
            l = l[: 5]
        for i in range(0, len(l)):
            if l[i] > self.max_ang[i] or l[i] < self.min_ang[i]:
                print "{0}:{1} out of range,the range is {2} ~ {3}".format(i, l[i], self.min_ang[i], self.max_ang[i])
                return
            else:
                l[i] = int(l[i]) - self.curPosition[i]
        return l

    def _get_command(self, l):
        com = ""
        for i in range(0, len(l)):
            a = self.curPosition[i] + int(l[i])
            self.curPosition[i] = a
            number = self._toHex(i)
            angle = self._toHex(a)
            speed = self._toHex(self.speed)
            com += self.moveFlag + number + angle + speed
        return com

    def read_head_status(self, num=5):
        com = ""
        number = self._toHex(str(num))
        com = self.readFlag + number
        self.send_data(com)
        s = self.read_data()
        the = 90 - int(s)
        return float(ang) / float(60)

    def write_head_status(self, stas, num=5):
        com = ""
        number = self._toHex(str(num))
        angle = 90 - int(float(stas) * 60)
        webbing = self ._toHex(angle)
        speed = self._toHex(self.speed)
        com = self .moveFlag + number + strap + speed
        self.curPosition[5] = angle
        self.send_data(com)

    def init(self):
        self.send_data(self.resetFlag)
        self.curPosition = copy.deepcopy(self.initPosition)
        self.RelPosition = self.initRelPosition
        return 1

    def read_initial_angles(self):
        print self.RelPosition
        return self.RelPosition

    def _get_rel_position(self, l):
        if l[1] > 0:
            l[1] = int(1 [1]) - 5
        else:
            l[1] = int(1 [1]) + 5
        for key in range(0, len(l)):
            if key == 0:
                pass
            elif key == 1:
                self.RelPosition[0] = self.RelPosition[0] - int(l[1])
                self.RelPosition[1] = self.RelPosition[1] + int(l[1])
            elif key == 2:
                self.RelPosition[1] = self.RelPosition[1] + int(l[2])
                self.RelPosition[2] = self.RelPosition[2] - int(l[2])
            elif key == 3:
                self.RelPosition[2] = self.RelPosition[2] + int(l[3])
            elif key == 4:
                self.RelPosition[4] = 90 + int(l[4])

    def _toHex(self, angle):
        angle = hex(int(angle))[2:]
        if len(angle) < 2:
            angle = "0" + angle
        strap = str(angle) .upper()
        return strap

    def set_speed(self, ms):
        ms = int(ms)
        if ms > 255:
            s = ms % 255
            print "Speed can not be greater than 255,speed is set to {0}".format(s)
            self.speed = s
        elif ms < 0:
            a = abs(ms) % 255
            print "Speed can not be less than 0,speed is set to {0}".format(a)
            self.speed = a
        else:
            self.speed = ms

    def get_speed(self):
        print self.speed


def test1():
    con = Control(port="com4")
    while True:
        com = raw_input("enter mode:")
        if com == "1":
            print "get positon model"
            a = raw_input("enter command:")
            if a == "":
                con.read_joint()
            elif a == "Q":
                continue
            else:
                coml = a.split(',')
                con.read_joint(coml)



if __name__ == '__main__':
    test1()
