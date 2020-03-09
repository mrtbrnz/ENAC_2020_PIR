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
# EXAMPLE, set the PWM to 10% and checks if the fans are running at correct RPM for 10 sec
#------------------------------------------------------------------------------

# Enable RPM counting on the module's boards
wcapi.startRPMFeature()

# Create a new log folder and starts logging, files accessible at 192.168.88.40/logs
wcapi.startLogOBC()

# Start a single Power Supply Unit (PSU)
wcapi.startPSUs()
time.sleep(1)

#start all fans t0 10%
wcapi.setPWM(10)
time.sleep(1)

t_start = time.time()
while time.time()-t_start < 10:
	print(wcapi.rpmVSpwm(fan_layer=0, acc_range=0.6))
	time.sleep(1)

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


