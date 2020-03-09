#!/usr/bin/env python

from labjack import ljm
import math, sys, numpy as np, time, threading, logging
import utility_functions as uf




class Labjack():
    def __init__(self):
        # Open first found LabJack
        self.handle = ljm.open(ljm.constants.dtANY, ljm.constants.ctANY, "ANY")
        #self.handle = ljm.openS("ANY", "ANY", "ANY")
        self.info = ljm.getHandleInfo(self.handle)
        print("Opened a LabJack with Device type: %i, Connection type: %i,\n" \
            "Serial number: %i, IP address: %s, Port: %i,\nMax bytes per MB: %i" % \
            (self.info[0], self.info[1], self.info[2], ljm.numberToIP(self.info[3]), self.info[4], self.info[5]))
        self.configure()


    def configure(self):
        # Setup and call eWriteNames to configure AINs on the LabJack.
        settling_us_all = 100 # 10^-6 sec
        numFrames = 30
        names = ["AIN0_NEGATIVE_CH", "AIN0_RANGE", "AIN0_RESOLUTION_INDEX", "AIN0_SETTLING_US",
                "AIN2_NEGATIVE_CH", "AIN2_RANGE", "AIN2_RESOLUTION_INDEX", "AIN2_SETTLING_US",
                "AIN4_NEGATIVE_CH", "AIN4_RANGE", "AIN4_RESOLUTION_INDEX", "AIN4_SETTLING_US",
                "AIN6_NEGATIVE_CH", "AIN6_RANGE", "AIN6_RESOLUTION_INDEX", "AIN6_SETTLING_US",
                "AIN8_NEGATIVE_CH", "AIN8_RANGE", "AIN8_RESOLUTION_INDEX", "AIN8_SETTLING_US",
                "AIN10_NEGATIVE_CH", "AIN10_RANGE", "AIN10_RESOLUTION_INDEX", "AIN10_SETTLING_US",
                "AIN12_RANGE", "AIN12_RESOLUTION_INDEX", "AIN12_SETTLING_US",
                "AIN13_RANGE", "AIN13_RESOLUTION_INDEX", "AIN13_SETTLING_US"]

        aValues = [ 1, 10, 1, settling_us_all,
                    3, 10, 1, settling_us_all,
                    5, 10, 1, settling_us_all,
                    7, 10, 1, settling_us_all,
                    9, 10, 1, settling_us_all,
                   11, 10, 1,settling_us_all,
                   10, 1, settling_us_all,
                   10, 1, settling_us_all]

        ljm.eWriteNames(self.handle, numFrames, names, aValues)

        print("\nSet configuration:")
        for i in range(numFrames):
            print("    %s : %f" % (names[i], aValues[i]))

    def get_signals(self):    
        # Setup and call eReadNames to read AINs from the LabJack.
        numFrames = 8
        names = ["AIN0", "AIN2", "AIN4", "AIN6", "AIN8", "AIN10", "AIN12", "AIN13"]
        TTM = uf.ttm(0.0,0.0,0.0,0.0,0.0,-48.0)
        self.signals = ljm.eReadNames(self.handle, numFrames, names)
        # self.R = uf.signal_to_force(self.signals)
        # F_TTM = TTM.dot(self.R)
        # self.R = F_TTM
        self.R = self.signals
        print("%f  %f  %f  %f  %f  %f  %f  %f  %f " % (time.time(), self.R[0], self.R[1], self.R[2], self.R[3], self.R[4], self.R[5], self.signals[6], self.signals[7]) )
        return time.time(), self.R[0], self.R[1], self.R[2], self.R[3], self.R[4], self.R[5]

    def clear(self):
        ljm.close(self.handle)

##############################




log_filename = sys.argv[1] if len(sys.argv)>1 else '/tmp/log'
max_meas = 100000

measurements = np.zeros((max_meas, 7))

labjack = Labjack()
i = 0
j = 0
dt = 0.02
while i < 30:
    # labjack.get_signals()
    measurements[j,0:7] = labjack.get_signals()
    time.sleep(dt)
    i += dt
    j += 1

with open(log_filename+"_s.txt", 'wb') as f:
    np.save(f, measurements[:j])

labjack.clear()

#EOF



