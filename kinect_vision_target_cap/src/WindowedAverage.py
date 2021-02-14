# Author: Trevor Sherrard
# Course: Directed Research
# Since: Febuary 14, 2021
# Description: This file contains the class definition for the 1D
#              windowed average data stream smoother. 

class MovingWindowAverage:
    def __init__(self, windowSize):
        self.windowSize = windowSize
        self.windowList = []
        self.rollingSum = 0

    def addAndProc(self, newVal):
        self.windowList.append(newVal)
        self.rollingSum += newVal
        numVals = len(self.windowList)

        # if the current list excedes the windowsize
        # remove the oldest element from running
        # sum and value list
        if(numVals > self.windowSize):
            oldestVal = self.windowList.pop(0)
            self.rollingSum -= oldestVal

        currentAvg = float(self.rollingSum) / numVals
        return currentAvg
