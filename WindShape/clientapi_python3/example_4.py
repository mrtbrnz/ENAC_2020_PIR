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
import time, datetime
from pathlib2 import Path

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
# EXAMPLE, turn on the first fan of each module to 10% for 10 sec
# This example calls all the modules (modules_flat) that are listed in the unique fan_wall (fan_Walls[0])
# that is found in the facility.
#------------------------------------------------------------------------------

# Enable RPM counting on the module's boards
wcapi.startRPMFeature()

# Create a new log folder and starts logging, files accessible at 192.168.88.40/logs
wcapi.startLogOBC()

# Start a single Power Supply Unit (PSU)
wcapi.startPSUs()
time.sleep(1)

pwm = 10

# Create a PWM string where the first fan unit (upwind and downwind fans) is set to 10%
pwm_str = "{},{},0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0".format(pwm, pwm)

# Turn on the first fan unit of each module

# with shortcuts:
# for module in wcapi.fan_wall.modules_flat:

# without shortcurs:
for module in wcapi.facility.fan_walls[0].modules_flat:
	module.setPWMstr(pwm_str)

time.sleep(10)


wcapi.stopLogOBC()

wcapi.stopRPMFeature()

#------------------------------------------------------------------------------
# PROPERLY STOP APP
#------------------------------------------------------------------------------

# Turn off the fans and the PSUs
wcapi.shutdown()

# Release the token 
# (will be released automatically if the app is closed)
wcapi.releaseToken()

# Terminate the communication threads 
# (will be terminated automatically if the app is closed)
wcapi.stopServerLink()


