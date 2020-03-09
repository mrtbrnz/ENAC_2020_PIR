#!/usr/bin/env python3
from labjack import ljm
import math, sys, numpy as np, time, threading, logging
import utility_functions as uf
from collections import deque
from ENAC_WS_libs import Labjack, pwm, bound, HEBI, start_Windshape, stop_Windshape
from scipy import interpolate
# import esc32 #, esc_can_class

# Windshape
import src.windControlAPI
import time

# Signal Library
from scipy import interpolate
from scipy import zeros, signal, random
from scipy import linalg as la

def step(t, a=-1, p=10., dt=0.): return a if math.fmod(t+dt,p) > p/2 else -a
def step_sin(t,duration,amp=1., nr=1): return amp*np.sin(2*nr*np.pi*t/duration)

def main_control():
    # Get the first CL argument as a output filename 
    log_filename = sys.argv[1] if len(sys.argv)>1 else '/tmp/log'
    
    # AoA_ref = 25./57.3  # inn radians.
    # AoA_sum_err = 0.0
    # AoA_last_err = 0.0 # initial for derivation
    # AoA_k_p = 55.0
    # AoA_k_d = 0.4
    # AoA_k_i = 10.0

    lift_ref = 6.0 # N
    pitch_sum_err = 0.0
    pitch_last_err = 0.0
    pitch_k_p = 3.0
    pitch_k_d = 0.2
    pitch_k_i = 4.0

    drag_ref = 0.0 # !
    throttle_sum_err = 0.0
    throttle_k_p = 15.0
    throttle_k_i = 8.

    # Flap Servo Mapping
    pwm = np.array([1250, 1575, 1910])
    servo_ang = np.array([30., 0., -30.]) # Positive down
    flap_pwm_of_ang = interpolate.interp1d(servo_ang, pwm)

    # Motor Servo Mapping
    pwm = np.array([1330, 1620, 1910])
    servo_ang = np.array([-30., 0., 30.]) # Right wing, positive down as well...
    motor_pwm_of_ang = interpolate.interp1d(servo_ang, pwm)

    fs = 340                   # Sampling frequency
    t = 5                       # Sampling period
    dt = 1.0 / fs

    hebi_offset = 0.71  #[rad] zero allignment offset.

    max_meas = 100000000
    measurements = np.zeros((max_meas, 25))

    LOG.info('Setting the Filter Parameters')

    fs = 340
    order = 2
    cutoff = 5.0
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = signal.butter(order, normal_cutoff)
    z_l = z_d = signal.lfilter_zi(b, a)

    try:
        esc = esc32.ESC(port ='/dev/tty.usbmodem33', speed=230400 , sim_mode=False)
        time.sleep(1)
        LOG.info('ESC assigned')

        # Initialize probe measurement
        labjack = Labjack()
        LOG.info('Labjack assigned')
        time.sleep(1)

        # HEBI Init
        hebi = HEBI()
        LOG.info('Hebi assigned')
        time.sleep(1)

        # WindShape Init
        wcapi = start_Windshape()
        LOG.info('WindShape started')

        # for static cases:
        # AoA = -90.0 #deg
        # For dynamic cases:
        AoA = 0.0 #deg

        hebi.go_to_position(AoA/180.*3.1415 + hebi_offset)

        flap_deflection = 0.0
        labjack.set_flap_servo(flap_pwm_of_ang(flap_deflection))

        motor_deflection = 0.0
        labjack.set_motor_servo(motor_pwm_of_ang(motor_deflection))

        LOG.info('Acquiring the bias data...')
        # Get the desired Lift Value:
        sample_nr = 6000
        data = np.zeros((sample_nr, 7))
        for i in range(sample_nr):
            data[i,0:7] = labjack.get_signals()

        LOG.info('Averaging the samples...')
        bias = np.mean(data[:,1:7], axis=0, dtype=np.float64) # There are 7 data, the first one is the time, the rest is force and moment signals.
        # Avg_Forces = labjack.get_forces(Avg_sig[1:7])
        # Desired_lift = Avg_Forces[0] #Avg_Forces[0] +
        # print(Avg_Forces)
        LOG.info('Bias = %f',  bias)

        esc.start_motor()
        time.sleep(1)
        throttle_neutral = 25 #0 #10 #1000
        esc.set_duty(throttle_neutral)
        time.sleep(5)

        PWM_start = 50
        wcapi.setPWM(PWM_start) # 37-38 for 5m/s , 62 for 7.5m/s , 82 for 10m/s
        time.sleep(10)

        j=0

        start_time = time.time()
        while time.time()-start_time < 120 :
            t = time.time()-start_time
            wcapi.setPWM(PWM_start-int(step_sin(t, 120, amp=49., nr=1 )))
            # if t<100:
            #     wcapi.setPWM(PWM_start-int(t/2))
            # else:
            #     wcapi.setPWM((PWM_start-50)+int((t-100)/2))

            measurements[j,0:7] = labjack.get_signals()
            Forces = labjack.get_forces(measurements[j,1:7], bias=bias)
            Fx , Fy = Forces[0:2,0]
            AoA = ((labjack.get_aoa()*72.)-46.)/162.159 # npw in rad # /2.8333 for deg
            pos = hebi.get_current_position()-hebi_offset
            
            Lift =  Fy*np.cos(AoA) + Fx*np.sin(AoA)
            Drag =  Fy*np.sin(AoA) - Fx*np.cos(AoA)

            Lift_f, z_l = signal.lfilter(b, a, [Lift], zi=z_l)
            Drag_f, z_d = signal.lfilter(b, a, [Drag], zi=z_d)


            # Lift Steps for tunning :
            # lift_ref = 1+ step(t, a=-1)

            lift_err = lift_ref - Lift_f
            drag_err = drag_ref - Drag_f

            throttle_err = lift_err*np.sin(AoA) - drag_err*np.cos(AoA)
            throttle_sum_err += throttle_err*dt
            throttle = throttle_neutral + throttle_err*throttle_k_p + throttle_sum_err*throttle_k_i
            throttle = bound(throttle, 6., 85.)
            # print('Throttle : ', int(throttle))
            esc.set_duty(int(throttle))
            

            # pitch_err = -lift_err
            pitch_err = -lift_err*np.cos(AoA) - drag_err*np.sin(AoA)
            pitch_err_d = (pitch_err-pitch_last_err)/dt
            pitch_last_err = pitch_err
            pitch_sum_err += pitch_err*dt
            flap_deflection = 0.0 + pitch_err*pitch_k_p  + pitch_err_d*pitch_k_d + pitch_sum_err*pitch_k_i


            # Step tests:
            # AoA_ref = np.deg2rad(7.5+step(t, a=-7.5))

            # # Control by Lift :
            # AoA_ref = lift_err*lift_k_p + lift_sum_err*lift_ki
            # # print('Lift sum err :', lift_sum_err, 'AoA ref [disp. deg]: ', AoA_ref*57.3)
            # print('Position = ', pos, ' AoA = ', AoA*57.3, 'Lift, Drag = ',Lift, Drag)
            # AoA_err = AoA - AoA_ref
            # AoA_err_d = (AoA_err - AoA_last_err)/dt
            # AoA_last_err = AoA_err
            # AoA_sum_err += dt*AoA_err
            # flap_deflection = 0.0 + AoA_err*AoA_k_p + AoA_err_d*AoA_k_d + AoA_sum_err*AoA_k_i

            # print(bound(flap_deflection, -30., 30.))
            labjack.set_flap_servo(flap_pwm_of_ang(bound(flap_deflection, -30., 30.)))

            # print('Lift = ', Lift)
            measurements[j,7:13] = Forces[:,0]
            measurements[j,13] = Lift_f
            measurements[j,14] = Drag_f

            measurements[j,15] = AoA
            measurements[j,16] = flap_deflection
            measurements[j,17] = throttle 
            measurements[j,18] = pitch_err
            measurements[j,19] = pitch_sum_err
            measurements[j,20] = throttle_err
            measurements[j,21] = throttle_sum_err

            measurements[j,22] = Lift
            measurements[j,23] = Drag
            measurements[j,24] = labjack.get_airspeed()

            j += 1
        print('Total smaple nr = ', j, 10/j)










    except KeyboardInterrupt:
        LOG.info('catched ctrl-C')
    finally:
        esc.stop_motor()
        esc.stop()
        hebi.clear() #group.clear_feedback_handlers()
        labjack.clear() #ljm.close(handle)
        stop_Windshape(wcapi)
        LOG.info('Cleared all parts...')
        with open(log_filename+".txt_b", 'wb') as f:  # FM: Force Moment
            np.save(f, measurements[:j])
        exit


# def main_wt():
#     # Get the first CL argument as a output filename 
#     log_filename = sys.argv[1] if len(sys.argv)>1 else '/tmp/log'
    
#     # Flap Servo Mapping
#     pwm = np.array([1250, 1575, 1910])
#     servo_ang = np.array([30., 0., -30.]) # Positive down
#     flap_pwm_of_ang = interpolate.interp1d(servo_ang, pwm)

#     # Motor Servo Mapping
#     pwm = np.array([1330, 1620, 1910])
#     servo_ang = np.array([-30., 0., 30.]) # Right wing, positive down as well...
#     motor_pwm_of_ang = interpolate.interp1d(servo_ang, pwm)

#     feq = 600                   # Sampling frequency
#     t = 5                       # Sampling period
#     dt = 1.0 / feq

#     hebi_offset = 0.71  #[rad] zero allignment offset.

#     max_meas = 100000000
#     measurements = np.zeros((max_meas, 14))

#     try :
#         # esc = esc_can_class.ESC_CAN(port='/dev/tty.usbmodem1431')
#         esc = esc32.ESC(port ='/dev/tty.usbmodem14531', speed=230400 , sim_mode=False)
#         time.sleep(1)
#         print('ESC assigned')

#         # Initialize probe measurement
#         labjack = Labjack()
#         print('Labjack assigned')
#         time.sleep(1)

#         # HEBI Init
#         hebi = HEBI()
#         print('Hebi assigned')
#         time.sleep(1)

#         # WindShape Init
#         wcapi = start_Windshape()
#         print('WindShape started')

#         # for static cases:
#         # AoA = -90.0 #deg
#         # For dynamic cases:
#         AoA = 0.0 #deg

#         hebi.go_to_position(AoA/180.*3.1415 + hebi_offset)

#         flap_deflection = 0.0
#         labjack.set_flap_servo(flap_pwm_of_ang(flap_deflection))

#         motor_deflection = 0.0
#         labjack.set_motor_servo(motor_pwm_of_ang(motor_deflection))


#         # esc.start_motor()
#         # time.sleep(1)
#         # throttle = 0 #0 #10 #1000
#         # esc.set_duty(throttle)
#         # time.sleep(5)

#         j=0
        

#         s = input('Is the windtunnel on ? Should we start ?\n')

#         print("======================")
#         print("**** Starting... *****")
#         print("======================")

#         # # This is a 2 hour test :
#         # AoA_array = np.arange(-5,15,5)     # 4 stations 
#         # flap_array = np.arange(-30, 40, 10) # 7 stations
#         # motor_ang_array = np.arange(-30, 40, 10) # 7 stations
#         # throttle_array = np.arange(10, 60, 10) # 5 stations

#         # Initial test
#         # AoA_array = np.arange(20,50,10)#(-7,28,7)#(-90,-85,5)#(-5,10,5)#(-90,-85,5)           # 3 stations 
#         # flap_array = np.arange(-30, 40, 10)      # 5 stations
#         # motor_ang_array = np.arange(-30, 40, 10) # 7 stations
#         # throttle_array = np.arange(0,49,7)#(0,60,10) #(10,60,10)#(0,10,10) #(10, 60, 30)   # 5 stations
#         # For 10m/s
#         AoA_array = np.concatenate([np.arange(0,90,0.15),np.arange(90,0,-0.3)])#(-7,28,7)#(-90,-85,5)#(-5,10,5)#(-90,-85,5)           # 3 stations 
#         flap_array = np.arange(0, 10, 10)      # 5 stations
#         motor_ang_array = np.arange(0, 10, 10) # 7 stations
#         throttle_array = np.arange(0,10,10)#(0,60,10) #(10,60,10)#(0,10,10) #(10, 60, 30)   # 5 stations

#         print("\n**** Taking zero load measurement... *****")
#         start = time.time()
#         while time.time() - start < 10 :
#             # hebi.go_to_position(-position/180.*3.1415 + hebi_offset)
#             measurements[j,0:7] = labjack.get_signals()
#             # measurements[j,0] -= start
#             measurements[j,7] = 0. #flap_deflection
#             measurements[j,8] = 0. #motor_deflection
#             measurements[j,9:12] = esc.get_current_measurement()
#             measurements[j,12] = -10. #throttle
#             measurements[j,13] = hebi.get_current_position()-hebi_offset
#             j+=1

#         wcapi.setPWM(0) # 37-38 for 5m/s , 62 for 7.5m/s , 82 for 10m/s
#         time.sleep(10)

#         for A_ in AoA_array:
#             hebi.go_to_position(A_/180.*3.1415 + hebi_offset)
#             # esc.set_duty(0)
#             # wcapi.setPWM(0)
#             # time.sleep(5)
#             # # get_zero_measurements()
#             # labjack.set_flap_servo(flap_pwm_of_ang(0))
#             # labjack.set_motor_servo(motor_pwm_of_ang(0))
#             # time.sleep(5)
#             # start = time.time()
#             # nextStart = start + dt
#             # while time.time() - start < 8 :
#             #     # hebi.go_to_position(-position/180.*3.1415 + hebi_offset)
#             #     measurements[j,0:7] = labjack.get_signals()
#             #     # measurements[j,0] -= start
#             #     measurements[j,7] = 0. #flap_deflection
#             #     measurements[j,8] = 0. #motor_deflection
#             #     measurements[j,9:12] = esc.get_current_measurement()
#             #     measurements[j,12] = -10. #throttle
#             #     measurements[j,13] = hebi.get_current_position()-hebi_offset
#             #     j+=1
#             #     #print(measurements[j:])
#             #     SleepTime = nextStart - time.time()
#             #     if SleepTime < 0 :
#             #         nextStart += dt
#             #         # print('Not able to keep up with the sampling rate... :( ')
#             #         continue
#             #     else :
#             #         time.sleep(SleepTime)
#             #         nextStart += dt

#             # wcapi.setPWM(82) # 37-38 for 5m/s , 62 for 7.5m/s , 82 for 10m/s
#             # time.sleep(10)
#             for flap_ in flap_array:
#                 labjack.set_flap_servo(flap_pwm_of_ang(flap_))
#                 for motor_ang in motor_ang_array:
#                     labjack.set_motor_servo(motor_pwm_of_ang(motor_ang))
#                     for throttle_ in throttle_array:
#                         # print('Iteration nr:', j, '/', len(AoA_array)*len(flap_array)*len(motor_ang_array)*len(throttle_array)*feq*t)
#                         # esc.start_motor()
#                         # esc.set_duty(throttle_)
#                         # time.sleep(2.5)


#                     # start = time.time()
#                     # nextStart = start + dt
#                     # while time.time() - start < t :
#                         # hebi.go_to_position(-position/180.*3.1415 + hebi_offset)
#                         measurements[j,0:7] = labjack.get_signals()
#                         # measurements[j,0] -= start
#                         measurements[j,7] = flap_
#                         measurements[j,8] = motor_ang
#                         measurements[j,9:12] = esc.get_current_measurement()
#                         measurements[j,12] = throttle_
#                         measurements[j,13] = hebi.get_current_position()-hebi_offset
#                         j+=1
#                         #print(measurements[j:])

                            


#         with open(log_filename+".txt", 'wb') as f:  # FM: Force Moment
#             np.save(f, measurements[:j])
#             #pdb.set_trace()
#         esc.set_duty(0)
#         hebi.go_to_position(hebi_offset)
#         time.sleep(5)
#         print('Finished data acquisition and returned back to zero position!')

#     except KeyboardInterrupt:
#         print('catched ctrl C')
#     finally:
#         esc.stop_motor()
#         esc.stop()
#         hebi.clear() #group.clear_feedback_handlers()
#         labjack.clear() #ljm.close(handle)
#         stop_Windshape(wcapi)
#         print('Cleared all parts...')
#         with open(log_filename+".txt_immergence", 'wb') as f:  # FM: Force Moment
#             np.save(f, measurements[:j])
#         exit


if   __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    LOG = logging.getLogger(__name__)
    main_control()

#EOF











