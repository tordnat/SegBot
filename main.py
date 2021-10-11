import odrive
from odrive.enums import *
import time
import math

print("finding odrive...")

board = odrive.find_any()
axis0 = board.axis0
axis1 = board.axis1
print("Bus voltage is " + str(board.vbus_voltage) + "V")



#Turn of is USB is disconnected
axis0.config.enable_watchdog = True
axis1.config.enable_watchdog = True
axis0.config.watchdog_timeout = 1 #Stop after 1 second of timeout
axis1.config.watchdog_timeout = 1

def feed_watchdog():
    axis0.watchdog_feed()
    axis1.watchdog_feed()


#print("Checking for existing calibration")
#board.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
#board.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

#Turning one rotation: Assumes pos control
#board.axis0.controller.input_vel = 3.14
#board.axis1.controller.input_pos = 3.14

#Position control: Very unstable
axis0.controller.config.input_mode = INPUT_MODE_POS_FILTER
axis1.controller.config.input_mode = INPUT_MODE_POS_FILTER
#axis0.controller.input_pos = 1
t0 = time.monotonic()

def vel_sinusoid_test():
    while True:
        feed_watchdog()
        target_vel = 5*math.sin((time.monotonic() - t0)/2)
        print("Target" + str(int(target_vel)))
        axis0.controller.input_vel = target_vel
        axis1.controller.input_vel = -axis0.controller.input_vel

def pos_sinusoid_test():
    while True:
        feed_watchdog()
        setpoint = 4.0 * math.sin((time.monotonic() - t0)*2)
        print(str(int(setpoint)))
        axis0.controller.input_pos = setpoint
        axis1.controller.input_pos = setpoint
        time.sleep(0.01)
pos_sinusoid_test()