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
from math import sin, cos, exp, sqrt

class WindFunction(object):
	def __init__(self, literal_function, min=0, max=100):
		self.literal_function = literal_function
		self.min = min
		self.max = max

	def calculate(self, x=0, y=0, t=0):
		exec("result="+str(self.literal_function))
		if result < self.min:
			return self.min
		elif result > self.max:
			return self.max
		return result
