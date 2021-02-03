#!/usr/bin/python

# Author: Trevor Sherrad
# Course: Directed Research
# Since: Feburary 3rd, 2020
#
# Description: This file defines the class that will be used to manage
#              the serial connection between the arduino nano reciever and 
#              the rest of the ROS system

import serial

class SerialManager:
    def __init__(self, port, baud):
        self.port = port
        self.baud = baud
        self.serObj = None

    def open(self):
        # construct serial object
        serObj = serial.Serial()

        # set individual object fields
        serObj.port = self.port
        serObj.baudrate = self.baud
        serObj.byteSize = serial.EIGHTBITS
        serObj.parity = serial.PARITY_NONE
        serObj.stopbits = serial.STOPBITS_ONE

        # attempt to open port
        try:
            serObj.open()
        except Exception, err:
            print("error opening serial port: %s" % err)

        # check if serial is open
        res = serObj.isOpen()

        if(res):
            self.serObj = serObj
            return res
        else:
            self.serObj = None
            return res

    def read(self):
        # try to read from the serial object
        lineFromSer = self.serObj.readline()

        return lineFromSer

    def close(self):
        # close the serial port
        self.serObj.close()
