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

# Python 3.7
from .connectedObjects import ConnectedObjects

class Client(ConnectedObjects):
	def __init__(self, facility, ip, client_id, name):
		super(Client, self).__init__()
		self.facility = facility
		self.ip_addr = ip
		self.id = client_id
		self.name = name
		self.life_points = self.facility.max_life_points
		self.is_connected = 1
		self.token_request = 0
		self.in_charge = 0 # Usefull only for client side

		self.cmd_powered = 0
		self.cmd_send_rpm = 0

	def decayLifePoints(self):
		super(Client, self).decayLifePoints()
		if self.life_points == 0:
			self.facility.removeClient(self.ip_addr)
			self.__del__()

	def __del__(self):
		self.facility.addToPrintBuff("[CLIENT] Client with IP : " + self.ip_addr + "deleted.")
