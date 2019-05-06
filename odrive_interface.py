'''
Originally copied from neomanic.
Updates to the functionalities
by san-soucie (John) and Sandra,
based off of the code from 
DGonz.
'''

import serial
from serial.serialutil import SerialException

import sys
import time
import logging
import traceback

import odrive
from odrive.enums import *

import fibre

pi = 3.1415927
in2mm = 25.4
mm2in = 1/in2mm
in2m = in2mm/1000

Nm2A = 0.00000604 #N*m/radian to A/count
#https://www.wolframalpha.com/input/?i=(1+N*m%2Fradian)*(2*pi+radians%2F400000)*(1%2F(2.6+N*m%2FA))

zeroVec = [[[0,0],[0,0]]]
offsets = [[[-8.59,-6.11],[-3.61,5.89]]]
thtDesired = [[[0,0],[0,0]]]
velDesired = [[[0,0],[0,0]]]
kP = [[[0,0],[0,0]]]
kD = [[[0,0],[0,0]]]
home_kp = [[[0,0],[0,0]]]
home_kd = [[[0,0],[0,0]]]
kPd = [[[0,0],[0,0]]]
kDd = [[[0,0],[0,0]]]

CPR2RAD = (2*math.pi/400000)

default_logger = logging.getLogger(__name__)
default_logger.setLevel(logging.DEBUG)

# create console handler and set level to debug
ch = logging.StreamHandler()
ch.setLevel(logging.DEBUG)

default_logger.addHandler(ch)

class ODriveFailure(Exception):
    pass

class ODriveInterfaceAPI(object):
    driver = None
    encoder_cpr = 4096
    right_axis = None
    left_axis = None
    connected = False
    _prerolled = False
    offset = 100000 # This must be changed according to the offset
    # we get from the limit switches that we install.
    #engaged = False
    axis0 = None
    axis1 = None
    axis2 = None
    odrvs = [None, None]
    usb_serials = ['2087377E3548', '2086378C3548']

    def __init__(self, logger=None):
        self.logger = logger if logger else default_logger

    def __del__(self):
        self.disconnect()

    def connect(self, port=None, right_axis=0, timeout=30, serial_number=None):
        #print("for serious>?!")
        if self.driver:
            self.logger.info("Already connected. Disconnecting and reconnecting.")
        try:
            print("Connected, dawg")
            self.driver = odrive.find_any(timeout=timeout, logger=self.logger, serial_number=serial_number)
            self.axes = (self.driver.axis0, self.driver.axis1) 
        except:
            self.logger.error("No ODrive found. Is device powered?")
            return False

        # save some parameters for easy access
        self.right_axis = self.driver.axis0 if right_axis == 0 else self.driver.axis1
        self.left_axis  = self.driver.axis1 if right_axis == 0 else self.driver.axis0
        self.encoder_cpr = self.driver.axis0.encoder.config.cpr

        self.connected = True
        self.logger.info("Connected to ODrive. Hardware v%d.%d-%d, firmware v%d.%d.%d%s" % (
            self.driver.hw_version_major, self.driver.hw_version_minor, self.driver.hw_version_variant,
            self.driver.fw_version_major, self.driver.fw_version_minor, self.driver.fw_version_revision,
            "-dev" if self.driver.fw_version_unreleased else ""
        ))
        return True
    
    def connect_all(self):
        for i in range(len(self.usb_serials)):
            self.odrvs[i] = self.connect(serial_number=self.usb_serials[i])
        self.axis0 = self.odrvs[0].axis0
        self.axis1 = self.odrvs[0].axis1
        self.axis2 = self.odrvs[1].axis0 # Or is this axis1? Good idea to check.
        self.axes = [self.axis0, self.axis1, self.axis2]

    def disconnect(self):
        self.connected = False
        self.right_axis = None
        self.left_axis = None
        self._prerolled = False
        #self.engaged = False
        if not self.driver:
            self.logger.error("Not connected.")
            return False
        try:
            self.release()
        except:
            self.logger.error("Error in timer: " + traceback.format_exc())
            return False
        finally:
            self.driver = None
        return True

    def calibrate(self):
        '''
        Should now be able to calibrate all three motors
        '''
        if not self.driver:
            self.logger.error("Not connected.")
            return False
        self.logger.info("Vbus %.2fV" % self.driver.vbus_voltage)
        for i, axis in enumerate(self.axes):
            self.logger.info("Calibrating axis %d..." % i)
            axis.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
            time.sleep(1)
            while axis.current_state != AXIS_STATE_IDLE:
                time.sleep(0.1)
            if axis.error != 0:
                self.logger.error("Failed calibration with axis error 0x%x, motor error 0x%x" % (axis.error, axis.motor.error))
                return False
        return True

    def release(self):
        if not self.driver:
            self.logger.error("Not connected.")
            return False
        self.logger.debug("Releasing.")
        for axis in self.axes: 
            axis.requested_state = AXIS_STATE_IDLE
        return True
    
    def full_init(self):
        self.odrvs[0].config.brake_resistance = 0
        self.odrvs[1].config.brake_resistance = 0
        for axis in self.axes:
            axis.requested_state = AXIS_STATE_IDLE
            axis.motor.config.current_lim =3 
            axis.motor.config.pole_pairs = 4
            axis.controller.config.vel_limit = 600000 #50000 counts/second is 1/8 revolution per second
            # 0.0612 [(revolutions/second)/Volt], 400000 counts per revolution
            # Max speed is 1.35 Revolutions/second, or 539000counts/second
            axis.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
            axis.encoder.config.cpr = 4000
            axis.encoder.config.bandwidth = 1000
            axis.encoder.config.use_index = True
            axis.encoder.config.zero_count_on_find_idx = True
            #axis.encoder.config.idx_search_speed = 1
            axis.encoder.config.pre_calibrated = False
            #motor calibration current
            axis.motor.config.calibration_current = 5
            #axis state
            if(axis.motor.config.pre_calibrated == False):
                axis.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        print("Setup done, dawg.")
        kP_des = 2
        kD_des = 0.0003
        for axis in self.axes:
            axis.requested_state = AXIS_STATE_IDLE
            axis.motor.config.pre_calibrated = True
            axis.config.startup_encoder_index_search = True
            axis.config.startup_encoder_offset_calibration = True
            axis.controller.config.vel_gain = kD_des
            axis.controller.config.vel_integrator_gain = 0
            axis.controller.config.pos_gain = 2.0
            axis.controller.pos_setpoint = 0
            axis.controller.vel_setpoint = 0            
            axis.config.startup_closed_loop_control = True
        self.odrv[0].save_configuration()
        self.odrv[1].save_configuration()
        print("Saved, homie")
        try:
            self.odrv[0].reboot()
            self.odrv[1].reboot()
        except:
            print('Rebooted')
        time.sleep(0.25)
        # Remember to run connect() again!

    def drive(self, left_motor_val, right_motor_val):
        if not self.driver:
            self.logger.error("Not connected.")
            return
        for axis in self.axes:
            axis.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
            axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL            
        self.left_axis.controller.vel_setpoint = left_motor_val
        self.right_axis.controller.vel_setpoint = right_motor_val

    def drivePos(self, left_motor_pos, right_motor_pos):
        if not self.driver:
            self.logger.error("Not connected.")
            return
        for axis in self.axes:
            axis.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL
            axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        #print(self.left_axis.controller.pos_setpoint, self.right_axis.controller.pos_setpoint)
        self.left_axis.controller.pos_setpoint = left_motor_pos
        self.right_axis.controller.pos_setpoint = right_motor_pos
        #print(self.left_axis.controller.pos_setpoint, self.right_axis.controller.pos_setpoint)

    def trajMoveCnt(self, pos, vel=50000, acc=50000):
        for i in range(len(self.axes)):
            self.axes[i].controller.config.control_mode = CTRL_MODE_POSITION_CONTROL
            self.axes[i].requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            self.axes[i].trap_traj.config.vel_limit = vel #50000 = 1/8 rev per second
            self.axes[i].trap_traj.config.accel_limit = acc #50000 = 1/8 rev per second^2
            self.axes[i].trap_traj.config.decel_limit = acc
            self.axes[i].controller.move_to_pos = pos[i]
            print(self.axes[i].controller.pos_setpoint)
            
    def trajMovRad(self, pos, vel=2*pi/8, acc=2*pi/8):
        self.trajMoveCnt(self.rad2Count(pos), self.rad2Count(vel), self.rad2Count(acc))

    def rad2Count(self, angle):
        try:
            return [(self.offset-ang)/CPR2RAD for ang in angle]
        except TypeError:
            return (self.offset-angle)/CPR2RAD
        
    def count2Rad(self, count):
        try:
            return[(self.offset-cnt)*CPR2RAD for cnt in count]
        except TypeError:
            return (self.offset-count)*CPR2RAD
        
    def get_errors(self, clear=True):
        # TODO: add error parsing, see: https://github.com/madcowswe/ODrive/blob/master/tools/odrive/utils.py#L34
        if not self.driver:
            return None

        axis_error = self.axes[0].error or self.axes[1].error

        if clear:
            for axis in self.axes:
                axis.error = 0
                axis.motor.error = 0
                axis.encoder.error = 0
                axis.controller.error = 0

        if axis_error:
            return "error"