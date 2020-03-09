"""
Copyright (C) WindShape LLC - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential. 
                     _ _ _ _       _ _____ _               
                    | | | |_|___ _| |   __| |_ ___ ___ ___  
                    | | | | |   | . |__   |   | .'| . | -_| 
                    |_____|_|_|_|___|_____|_|_|__,|  _|___| 
                                                  |_|        
 
WindShape LLC, Geneva, February 2019 

Developer team:
	Guillaume Catry <guillaume.catry@windshape.ch>
	Nicolas Bosson <nicolas.bosson@windshape.ch>

Contributors:
	Adrien Fleury, Alejandro Stefan Zavala, Federico Conzelmann
"""

# Test with Python 3.6 and 3.7
import src.windControlAPI
import time

#------------------------------------------------------------------------------
# WINDCONTROL APP INITIALIZATION
#------------------------------------------------------------------------------

# Start WindControl App and initiate communication with the server
wcapi = src.windControlAPI.WindControlApp(verbose_comm=True)

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
time.sleep(1)

# Define a WindFunction
literal_function = "30*sin(3*x+2*t)+4*y*t"
myWindFunction = wcapi.defineWindFunction(literal_function, min=5, max=20)

duration = 10
dt = 0.04
wcapi.startWindFunction(myWindFunction, duration, dt)

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


