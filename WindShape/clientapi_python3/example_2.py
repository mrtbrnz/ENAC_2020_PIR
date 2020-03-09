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
wcapi = src.windControlAPI.WindControlApp(verbose_comm=False)

# Start communication with server
wcapi.startServerLink()

# Request the the control token (only one client can have the control)
wcapi.requestToken()

#------------------------------------------------------------------------------
# EXAMPLE, constant PWM and print RPMs for 10 sec
#------------------------------------------------------------------------------

# Enable RPM counting at the boards
wcapi.startRPMFeature()

# Create a new log folder and starts logging, files accessible at 192.168.88.40/logs
wcapi.startLogOBC()

# Start a single Power Supply Unit (PSU)
wcapi.startPSUs()
time.sleep(1)

# Start all fans to 10%
wcapi.setPWM(10)

t_start = time.time()
while time.time()-t_start < 10:
	print(wcapi.getRPM())
	time.sleep(1)

wcapi.setPWM(0)
time.sleep(3)

wcapi.stopLogOBC()

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


