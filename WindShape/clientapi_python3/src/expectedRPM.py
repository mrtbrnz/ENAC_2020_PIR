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
import matplotlib.pyplot as plt

def expectedRPM(pwm, fan_layer=0):
	delta_pwm = 5
	pwm_discret = [i*5 for i in range(100/delta_pwm)]

	rpm_u = [1406,
			2037,
			2730,
			3355,
			4087,
			4706,
			5354,
			6058,
			6620,
			7213,
			7801,
			8474,
			9158,
			9790,
			10463,
			11147,
			11800,
			12517,
			13174,
			13682]

	rpm_d = [1348,
			1968,
			2639,
			3244,
			3848,
			4521,
			5092,
			5776,
			6340,
			6904,
			7478,
			8124,
			8767,
			9412,
			10051,
			10696,
			11334,
			11982,
			12855,
			13147]

	rpm_exp = []
	for i in range(len(pwm_discret)-1):
		if pwm == pwm_discret[i]:
			rpm_u_exp = rpm_u[i]
			rpm_d_exp = rpm_d[i]
			rpm_m_exp = (rpm_u_exp+rpm_d_exp)/2
			break
		elif pwm >= pwm_discret[i] and pwm <= pwm_discret[i+1]:
			add_pwm = pwm-pwm_discret[i]
			rpm_u_exp = add_pwm*(rpm_u[i+1]-rpm_u[i])/delta_pwm+rpm_u[i]
			rpm_d_exp = add_pwm*(rpm_d[i+1]-rpm_d[i])/delta_pwm+rpm_d[i]
			rpm_m_exp = (rpm_u_exp+rpm_d_exp)/2
			break

	rpm_exp = [rpm_m_exp, rpm_u_exp, rpm_d_exp]

	return rpm_exp[fan_layer]