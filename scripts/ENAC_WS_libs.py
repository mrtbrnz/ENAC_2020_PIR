#!/usr/bin/env python3
# Libraries for Trisonik Lab

#HEBI
import os
os.environ['HEBI_C_LIB'] = '/Users/murat/Work/Temporary_Work/DAQ_Py/Hebi_Py/lib/libhebi.dylib'
from hebi import *

from labjack import ljm
import math, sys, numpy as np, time, threading, logging
import utility_functions as uf
from collections import deque

# Windshape
import src.windControlAPI

def start_Windshape():
    #------------------------------------------------------------------------------
    # WINDCONTROL APP INITIALIZATION
    #------------------------------------------------------------------------------

    # Start WindControl App and initiate communication with the server
    wcapi = src.windControlAPI.WindControlApp(verbose_comm=False)

    # Start communication with server
    wcapi.startServerLink()

    # Request the the control token (only one client can have the control)
    wcapi.requestToken()

    #------------------------------------------------------------------------------
    # EXAMPLE, run a random windfunction for 10 sec, with PWM min and max values set to 5 and 20%
    #------------------------------------------------------------------------------

    # Enable RPM counting on the module's boards
    wcapi.startRPMFeature()

    # Create a new log folder and starts logging, files accessible at 192.168.88.40/logs
    wcapi.startLogOBC()

    # Start Power Supply Unit (PSU)
    wcapi.startPSUs()
    time.sleep(3)
    return wcapi

def stop_Windshape(wcapi):
    # Stop all fans
    wcapi.setPWM(0)
    time.sleep(3)

    # Stop logging
    wcapi.stopLogOBC()

    # Disable RPM counting on the module's boards
    wcapi.stopRPMFeature()

    #------------------------------------------------------------------------------
    # PROPERLY STOP APP
    #------------------------------------------------------------------------------

    wcapi.shutdown()
    # Release the token 
    # (will be released automatically if the app is closed)
    wcapi.releaseToken()

    # Terminate the communication threads 
    # (will be terminated automatically if the app is closed)
    wcapi.stopServerLink()

class CircularBuffer_1(deque):
     def __init__(self, size=0):
             super(CircularBuffer, self).__init__(maxlen=size)
     @property
     def average(self):  # TODO: Make type check for integer or floats
             return sum(self)/len(self)
class CircularBuffer(object):
    def __init__(self, size):
        """initialization"""
        self.index= 0
        self.size= size
        self._data = []

    def record(self, value):
        """append an element"""
        if len(self._data) == self.size:
            self._data[self.index]= value
        else:
            self._data.append(value)
        self.index= (self.index + 1) % self.size

    def __getitem__(self, key):
        """get element by index like a regular array"""
        return(self._data[key])

    def __repr__(self):
        """return string representation"""
        return self._data.__repr__() + ' (' + str(len(self._data))+' items)'

    def get_all(self):
        """return a list of all the elements"""
        return(self._data)

def pwm(value):
    return int(value/20000.0*1600000.0)

def bound(value, low=1000, high=2000):
    # print(value)
    return max(low, min(high, value))

##############################
class HEBI():
    def __init__(self):
        # Open the HEBI group
        self.lookup = Lookup()
        self.group = self.lookup.get_group_from_names(['X5-1'], ['X-00425'])
        if not self.group:
            print('HEBI Group not found !')
            exit(1)

        # Init position
        self.position_rad = 0.7
        # global actual_position
        self.actual_position = 0.7
        self.run()

    def run(self):
         # Sets the command lifetime to 100 milliseconds
        self.group.set_command_lifetime_ms(25)
        self.group_command = GroupCommand(self.group.size) 
        self.group.add_feedback_handler(self.feedback_handler)
        self.position_rad = self.actual_position
        # Control the robot at 100Hz
        self.group.set_feedback_frequency(400)


    def update_position(self, value):
        #global self.actual_position
        self.actual_position = value

    def get_current_position(self):
        return self.actual_position

    def feedback_handler(self, group_fbk):
        # HEBI feedback Handler
        # Uncomment to print out the actual position
        #print("Position :", group_fbk.position)
        self.update_position(group_fbk.position)
        self.group_command.set_position([self.position_rad])
        # group_command.set_effort(spring_constant * group_fbk.position)
        self.group.send_command(self.group_command)


    def go_to_position(self, desired_position):
      # global position_rad
      # global actual_position
      # self.position_rad = desired_position # Just changed for trisonic, and uncomment the below stuff
      error = desired_position - self.actual_position
      while abs(error) > 0.02:
        new_position = self.actual_position + uf.bound(error, 0.005)
        self.position_rad = uf.bound_arm(new_position)
        #print(error, self.actual_position, new_position)
        time.sleep(0.02) # Make 0.05 for faster...
        error = desired_position - self.actual_position

    def clear(self):
        self.group.clear_feedback_handlers()
##############################

class Labjack():
    def __init__(self):
        # Open first found LabJack
        #self.handle = ljm.open(ljm.constants.dtANY, ljm.constants.ctANY, "ANY")
        self.handle = ljm.openS("ANY", "ANY", "ANY")
        self.info = ljm.getHandleInfo(self.handle)
        print("Opened a LabJack with Device type: %i, Connection type: %i,\n" \
            "Serial number: %i, IP address: %s, Port: %i,\nMax bytes per MB: %i" % \
            (self.info[0], self.info[1], self.info[2], ljm.numberToIP(self.info[3]), self.info[4], self.info[5]))
        print('Labjack Class initialization 1')
        self.configure()
        print('Labjack Class initialization 2')
        self.servo_position = 1500.0 # in PWM value units
        self.airspeed = 0.0
        self.flap_servo_neutral = 1580.
        self.motor_servo_neutral = 1620.
        print('Labjack Class Servo Configuration')
        self.configure_servo()



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
        print("!!!!!!! BE CAREFUL , YOU ARE USING ENAC CALIBRATION FILE !!!!!")
        for i in range(numFrames):
            print("    %s : %f" % (names[i], aValues[i]))

    def configure_servo(self):
        servo_neutral = 1500
        # Configure Clock Registers:
        ljm.eWriteName(self.handle, "DIO_EF_CLOCK0_ENABLE", 0)   # Disable clock source
        # Set Clock0's divisor and roll value to configure frequency: 80MHz/1/1600000 = 50Hz
        ljm.eWriteName(self.handle, "DIO_EF_CLOCK0_DIVISOR", 1)  # Configure Clock0's divisor
        ljm.eWriteName(self.handle, "DIO_EF_CLOCK0_ROLL_VALUE", 1600000) # Configure Clock0's roll value for 50Hz
        ljm.eWriteName(self.handle, "DIO_EF_CLOCK0_ENABLE", 1)   # Enable the clock source

        # Configure 2nd EF Channel Registers:
        ljm.eWriteName(self.handle, "DIO2_EF_ENABLE", 0)  # Disable the EF system for initial configuration
        ljm.eWriteName(self.handle, "DIO2_EF_INDEX", 0)   # Configure EF system for PWM
        ljm.eWriteName(self.handle, "DIO2_EF_OPTIONS", 0) # Configure what clock source to use: Clock0
        ljm.eWriteName(self.handle, "DIO2_EF_CONFIG_A", pwm(self.flap_servo_neutral)) # Configure duty cycle to so that PWM is 1500ms
        ljm.eWriteName(self.handle, "DIO2_EF_ENABLE", 1) # Enable the EF system, PWM wave is now being outputted

        # Configure 3rd EF Channel Registers:
        ljm.eWriteName(self.handle, "DIO3_EF_ENABLE", 0)  # Disable the EF system for initial configuration
        ljm.eWriteName(self.handle, "DIO3_EF_INDEX", 0)   # Configure EF system for PWM
        ljm.eWriteName(self.handle, "DIO3_EF_OPTIONS", 0) # Configure what clock source to use: Clock0
        ljm.eWriteName(self.handle, "DIO3_EF_CONFIG_A", pwm(self.motor_servo_neutral)) # Configure duty cycle to so that PWM is 1500ms
        ljm.eWriteName(self.handle, "DIO3_EF_ENABLE", 1) # Enable the EF system, PWM wave is now being outputted

    def set_flap_servo(self,value):
        ljm.eWriteName(self.handle, "DIO2_EF_CONFIG_A", pwm(float(value)))

    def set_motor_servo(self,value):
        ljm.eWriteName(self.handle, "DIO3_EF_CONFIG_A", pwm(float(value)))
        # print(pwm(value))
    # def update_position(self, value):
    #     #global self.actual_position
    #     self.servo_position = value

    # def get_current_position(self):
    #     return self.servo_position

    # def go_to_position(self, desired_position):
    #   # global position_rad
    #   # global actual_position
    #   # self.position_rad = desired_position # Just changed for trisonic, and uncomment the below stuff
    #   error = desired_position - self.servo_position
    #   while abs(error) > 0.05:
    #     new_position = self.servo_position + uf.bound(error, 0.01)
    #     self.position_rad = uf.bound_arm(new_position)
    #     #print(error, self.actual_position, new_position)
    #     time.sleep(0.1) # Make 0.05 for faster...
    #     error = desired_position - self.servo_position

    def get_aoa(self):
        return ljm.eReadName(self.handle, "AIN13")

    def get_signals(self):    
        # Setup and call eReadNames to read AINs from the LabJack.
        numFrames = 7
        names = ["AIN0", "AIN2", "AIN4", "AIN6", "AIN8", "AIN10", "AIN12"]
        self.signals = ljm.eReadNames(self.handle, numFrames, names)
        #self.R = uf.signal_to_force(self.signals)
        self.R = self.signals
        self.airspeed = self.signals[6]
        #print("%f  %f  %f  %f  %f  %f  %f  %f  " % (time.time(), self.R[0], self.R[1], self.R[2], self.R[3], self.R[4], self.R[5], self.signals[6]) )
        return time.time(), self.R[0], self.R[1], self.R[2], self.R[3], self.R[4], self.R[5]#, self.signals[6]
        
    def get_airspeed(self):
        return self.airspeed

    def get_forces(self, signals, bias=np.array([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])):
        self.F = uf.signal_to_force(self.signals , uf.C40_ENAC, bias=bias) #C17_ITU)
        #print("%f  %f  %f  %f  %f  %f  %f  %f  " % (time.time(), self.R[0], self.R[1], self.R[2], self.R[3], self.R[4], self.R[5], self.signals[6]) )
        return self.F #time.time(), self.F #[0], self.R[1], self.R[2], self.R[3], self.R[4], self.R[5]#, self.signals[6]

    def get_digital_input(self):
        # Setup and call eReadName to read the DIO state from the LabJack.
        io_name = "FIO2"
        return ljm.eReadName(self.handle, io_name)

    def write_digital_signal(self, state):
        # Setup and call eWriteName to set the DIO state on the LabJack.
        io_name = "FIO1"
        return ljm.eWriteName(self.handle, io_name, state)

    def clear(self):
        # Turn off PWM output and counter
        aNames = ["DIO_EF_CLOCK0_ENABLE", "DIO0_EF_ENABLE", "DIO2_EF_ENABLE", "DIO3_EF_ENABLE"]
        aValues = [0, 0, 0, 0]
        numFrames = len(aNames)
        self.set_flap_servo(self.flap_servo_neutral)
        self.set_motor_servo(self.motor_servo_neutral)
        time.sleep(0.7)
        results = ljm.eWriteNames(self.handle, numFrames, aNames, aValues)
        ljm.close(self.handle)

##############################