import odrive
from odrive.enums import *
import time
import math
import random


##Watchdog 
def watchdog_feed(drive):
    drive.axis0.watchdog_feed()

def clear_watchdog(drive):
    drive.axis0.requested_state = AXIS_STATE_IDLE ##Must set idle before clearing errors
    drive.axis0.config.enable_watchdog = False
    drive.axis0.config.watchdog_timeout = 0.5
    drive.clear_errors()
    drive.clear_errors()
    watchdog_feed(drive)
    drive.axis0.config.enable_watchdog = True
    watchdog_feed(drive)
    set_velocity_control(drive)

def set_velocity_control(drive):
    drive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    drive.axis0.controller.config.input_mode = INPUT_MODE_VEL_RAMP
    drive.axis0.controller.config.vel_ramp_rate = 40 #How agressive (m/s^3)

def get_sign(x):
    if (x == 0):
        return 1
    else:
        return x/abs(x)

def quick_const_test():
    board = odrive.find_any()
    board.axis0.config.enable_watchdog = False
    board.clear_errors()
    board.axis0.watchdog_feed()
    board.axis0.config.enable_watchdog = True
    board.axis0.watchdog_feed()
    board.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    board.axis0.controller.config.input_mode = INPUT_MODE_VEL_RAMP
    board.axis0.controller.config.vel_ramp_rate = 40 #How agressive (m/s^3)


    board.axis0.config.watchdog_timeout = 1

    while True:
        board.axis0.watchdog_feed()
        input_vel =  2*math.sin(time.time()*2)
        print("Setpoint: " + str(input_vel))
        board.axis0.controller.input_vel = input_vel


def const_vel(drive,vel):
    print("Setpoint: " + str(vel))
    while True:
        print("Inputvel:")
        watchdog_feed(drive)
        drive.axis0.controller.input_vel = vel


class event_handler():
    hazard = 5
    event_duration = 10
    def __init__(self, drive, error_threshold, power_threshold, filter, trigger_counter, event_timeout_t):
        self.drive = drive
        self.error_threshold = error_threshold
        self.power_threshold = power_threshold
        self.filter_t = filter
        self.trigger_counter = trigger_counter
        self.event_timeout_t = event_timeout_t
        self.last_event_t = time.time()
    
    def get_error(self):
        vel_estimate = self.drive.axis0.encoder.vel_estimate
        target_vel = self.drive.axis0.controller.input_vel
        error_vel = target_vel - vel_estimate
        return abs(error_vel)
    def get_power(self):
        return self.drive.axis0.controller.mechanical_power

    def is_error_above_threshold(self):
        return bool(self.get_error()>=self.error_threshold)

    def print_power(self):
        print("Power: " + str(self.get_power()))
    def is_power_above_threshold(self):
        return bool(self.get_power() >= self.power_threshold)


    def filter(self):
        print("Triggered:" + str(self.trigger_counter))
        if(self.is_power_above_threshold()):
            self.trigger_counter +=1 
            print("Above threshold")
        else:
            if(self.trigger_counter > 0):
                self.trigger_counter += -1
        if (self.trigger_counter > self.filter_t):
            self.trigger_counter = 0
            print("Above filter")
            return True
        return False

    def event_timeout(self):
        if((time.time() - self.last_event_t) > self.event_timeout_t):
            return False
        else:
            return True
            
    def event(self):
        watchdog_feed(self.drive)
        vel_sign = int(get_sign(self.drive.axis0.controller.input_vel))
        print("Direction:" + str(vel_sign))
        if (self.filter() and not(self.event_timeout())):
            print("Event triggered")
            print("Event will last " + str(self.event_duration) + " seconds")
            self.drive.clear_errors()
            t0 = time.time()
            wait = random.randint(0,5)
            print("Waiting: " + str(wait) + " seconds")
            self.last_event_t = t0 + self.event_duration + wait 
            delay = time.time() + self.event_duration + wait #How long we want the event to last
            self.drive.axis0.controller.input_vel = 0
            while(time.time() < t0 + wait):
                self.drive.axis0.requested_state = AXIS_STATE_IDLE ##Must set idle before clearing errors
                #Now robots can escape
                #Blubb()
                watchdog_feed(self.drive)
            self.drive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL ##Change back into closed loop
            while (time.time() < delay):
                self.drive.clear_errors()
                watchdog_feed(self.drive)
                self.drive.axis0.controller.input_vel = self.hazard*vel_sign




        

def robowars_hazard(drive,t0):
    #event_h = event_handler(drive=drive, error_threshold=0.5,power_threshold=2.2, filter=3, trigger_counter=0, event_timeout_t=55) 
    while True:
        drive.axis0.watchdog_feed()
        target_vel = 2*math.sin((time.monotonic() - t0)/2)
        #vel_estimate = drive.axis0.encoder.vel_estimate
        #error_vel = target_vel - vel_estimate

        #print("Target: " + str(float(target_vel)))
        #print("Error: " + str(error_vel))
        #event_h.event()
        #event_h.print_power()
        drive.axis0.controller.input_vel = target_vel



def __main__():
    ##Startup sequence
    print("finding odrive...")
    board = odrive.find_any()

    watchdog_feed(board)
    clear_watchdog(board)
    board.axis0.watchdog_feed()
    print("Bus voltage is " + str(board.vbus_voltage) + "V")
    board.axis0.watchdog_feed()
    t0 = time.monotonic() #Start time    

    ##Define constants
    #set_velocity_control(board)
    robowars_hazard(board,t0)
    #const_vel(board,2)

__main__()
#quick_const_test()




