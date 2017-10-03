"""
Minimize time to fly path

We evolve on discrete roll, pitch, yaw and then convert to quaternions for actual
flight.
"""

import logging.config
from dronekit import connect, Command, LocationGlobal, Vehicle, VehicleMode
from pymavlink import mavutil
import time, sys, argparse, math, os, yaml
import numpy as np
from tracker import *
import threading
from collections import deque

import abc 

from pyulog.core import ULog

import glob
import signal
import argparse
import copy

from deap import algorithms
from deap import base
from deap import creator
from deap import tools
from subprocess import call,Popen

from sim.gazebo.gazeboapi import GazeboAPI 
from track import * 

#ROLL = 1
#PITCH = 2
#YAW = 3
DT = 0
THRUST = 1
Q0 =2 
Q1 =3
Q2 =4
Q3 =5
ROLL = 2
PITCH = 3
YAW = 4

def init_logging():
    log_config = os.path.join(".", "../conf/logging.yaml")
    if os.path.exists(log_config):
        with open(log_config, 'rt') as f:
            log_config = yaml.load(f.read())
        logging.config.dictConfig(log_config)
    else:
        raise Exception("Cannot find log configuration file")

logger = logging.getLogger('fastautopilot')
loggerPX4 = logging.getLogger('PX4')

"""
def quaternion_to_eulerian(w, x, y, z):
    #Return the Eulerian in degrees
    ysqr = y*y
    
    t0 = +2.0 * (w * x + y*z)
    t1 = +1.0 - 2.0 * (x*x + ysqr)
    roll = math.degrees(math.atan2(t0, t1))
    
    t2 = +2.0 * (w*y - z*x)
    t2 =  1 if t2 > 1 else t2
    t2 = -1 if t2 < -1 else t2
    pitch = math.degrees(math.asin(t2))
    
    t3 = +2.0 * (w * z + x*y)
    t4 = +1.0 - 2.0 * (ysqr + z*z)
    yaw = math.degrees(math.atan2(t3, t4))
    
    return roll, pitch, yaw 
"""

def quaternion_to_eulerian(w, x, y, z):
    dcm = quaternion_to_dcm(w, x, y, z)
    return dcm_to_euler(dcm)

def eulerian_to_quaternion(roll, pitch, yaw):
    """ roll pitch yaw in radians
    Copied from https://github.com/mavlink/c_library_v1/blob/master/mavlink_conversions.h """
    cosPhi_2 = math.cos(roll * 0.5)
    sinPhi_2 = math.sin(roll * 0.5)
    cosTheta_2 = math.cos(pitch * 0.5)
    sinTheta_2 = math.sin(pitch * 0.5)
    cosPsi_2 = math.cos(yaw * 0.5)
    sinPsi_2 = math.sin(yaw * 0.5)

    w = (cosPhi_2 * cosTheta_2 * cosPsi_2 +
                                 sinPhi_2 * sinTheta_2 * sinPsi_2)
    x = (sinPhi_2 * cosTheta_2 * cosPsi_2 -
        cosPhi_2 * sinTheta_2 * sinPsi_2)
    y = (cosPhi_2 * sinTheta_2 * cosPsi_2 +
        sinPhi_2 * cosTheta_2 * sinPsi_2)
    z = (cosPhi_2 * cosTheta_2 * sinPsi_2 -
        sinPhi_2 * sinTheta_2 * cosPsi_2)
    return w, x, y, z
def quaternion_to_dcm(a, b, c, d):
    dcm = [[0]*3 for _ in range(3)] 
    aSq = a * a
    bSq = b * b
    cSq = c * c
    dSq = d * d
    dcm[0][0] = aSq + bSq - cSq - dSq
    dcm[0][1] = 2 * (b * c - a * d)
    dcm[0][2] = 2 * (a * c + b * d)
    dcm[1][0] = 2 * (b * c + a * d)
    dcm[1][1] = aSq - bSq + cSq - dSq
    dcm[1][2] = 2 * (c * d - a * b)
    dcm[2][0] = 2 * (b * d - a * c)
    dcm[2][1] = 2 * (a * b + c * d)
    dcm[2][2] = aSq - bSq - cSq + dSq

    return dcm

def dcm_to_euler(dcm):
    """ return roll pitch yaw in radians"""

    pitch = math.asin(-dcm[2][0])
    M_PI_2 = math.pi/2.0

    if (abs(pitch - M_PI_2) < 0.001): 
        roll = 0.0
        yaw = (math.atan2(dcm[1][2] - dcm[0][1],
                dcm[0][2] + dcm[1][1]) + roll)

    elif (abs(pitch + M_PI_2) < 0.001): 
        roll = 0.0
        yaw = math.atan2(dcm[1][2] - dcm[0][1],
                  dcm[0][2] + dcm[1][1] - roll)

    else: 
        roll =math.atan2(dcm[2][1], dcm[2][2])
        yaw = math.atan2(dcm[1][0], dcm[0][0])

    return roll, pitch, yaw


class ControlInput(object):
    def __init__(self, roll, pitch, yaw, thrust):
        """ Each parameter is an array of tuples (boot_ms, data_value)
        Since each control parameter can occur at a different time, each must be bound
        to its own time stamp
        """
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.thrust = thrust

class RaceEvent(object):

    def __init__(self, time, event):
        """ Normalized time since boot ms """
        self.time

        """ Text of event """
        self.event = event


class QuadrotorPX4(Vehicle):
    """
    Copter is contolled using SET_POSITION_TARGET_LOCAL_NED and SET_ATTITUDE_TARGET 

    https://github.com/PX4/Firmware/issues/1288

    For offboard,
    http://discuss.px4.io/t/offboard-automatic-takeoff-landing-using-mav-cmd-land-takeoff-local/1333/3

    If we are not always sending target location message it switches to RTL

    """
    __metaclass__ = abc.ABCMeta

    """ Rate to check if waypoint is hit in hertz """
    SET_PT_RATE = 20

    INIT_ALTITUDE = -2

    """
    MUST PUBLISH FASTER THAN  2HZ TO STAY IN OFFBOARD
    https://dev.px4.io/en/ros/mavros_offboard.html """
    OFFBOARD_MODE_RATE = 4

    TAKEOFF_JITTER = -0.1

    def __init__(self, *args):
        super(QuadrotorPX4, self).__init__(*args)

        # Track the trajectory
        self.flight_trajectory = []
        # Track the inputs
        self.flight_attitudes = []

        self.time_fc_usec = None
        self.time_boot_ms = None

        self.time_ms = None

        self.start_position = None

        self.time_since_takeoff = None

        self.time_takeoff = None
        
        self.time_since_armed = None

        self.time_armed = None

        """ The boot time retured by MAVLink is the time when we start PX4
        not from the MAVLink connection. The time_boot_ms is therefore 
        incrementing. This allows us to normalize between flights.
        """
        self.time_first = None

        self.time_start = None

        self.time_lap = None

        self.time_normalize = None

        self.threads = []

        """ Current mode to detect when we change """
        self.curr_mode = None

        self.running = False

        self.race_end = False

        self.record = False

        """ An array of RaceEvents to track vents occur, i.e., takeoff, land, pass gate """
        self.events = []

        self.out_of_bounds = False


        """ The track the MAV will fly """
        self.track = None

        """ Save to check for transitions, i.e., takeoff or land """
        self.curr_land_state = None

        """ Instance of FlightData """
        self.fight_data = None


        """ When gate_next == track.gate_count then all have been reached """
        self.gate_next = 0 

        self.gate_times = []

        """ Normalized off first sample """
        self.time_last_attitude_sample = None
        self.time_first_sample = None

        """
        Each message is sent at a certain rate as defined in configure_stream
        src/modules/mavlink/mavlink_main.cpp
        """

        @self.on_message('STATUSTEXT')
        def statustext_listener(self, name,text):
            
            # TODO map severity to python logger levels
            #loggerPX4.info(text)
            pass

        @self.on_message('SYSTEM_TIME')
        def system_time_listener(self, name, t):

            self.time_fc_usec = t.time_unix_usec
            self.time_boot_ms = t.time_boot_ms

            if not self.time_first:
                self.time_first = t.time_boot_ms

            self.time_ms = t.time_boot_ms - self.time_first 

            if self.armed and not self.time_armed:
                self.time_armed = t.time_boot_ms - self.time_first
                self.debug("Armed at {}".format(self.time_armed))

            if self.time_armed:
                self.time_since_armed = self.time_ms - self.time_armed 

            if self.time_takeoff:
                self.time_since_takeoff = self.time_ms - self.time_takeoff
                

        @self.on_message('HEARTBEAT')
        def heartbeat(self, name, beat):
            #self.info("HEARTBEAT")
            pass

        #@self.on_message('ATTITUDE')
        #def attitude_listener(self, name, attitude):
            #print attitude
        #    pass

        """
        @self.on_message('ATTITUDE_TARGET')
        def attitude_target_listener(self, name, target):
            #sys.stdout.write('.')

            #print "{} {}".format(target.time_boot_ms, target.thrust)
            #if self.armed and self.time_since_armed:# and self.record:
            #target.time_boot_ms = self.time_since_armed

            if not self.time_first_sample:
                self.time_first_sample = target.time_boot_ms

            #if self.time_takeoff:
            self.time_last_attitude_sample = target.time_boot_ms - self.time_first_sample
            target.time_boot_ms = self.time_last_attitude_sample 
            #self.flight_attitudes.append(target)
            #self.log_att(target)
            #print target

        @self.on_message('EXTENDED_SYS_STATE')
        def landed_state_listener(self, name, state):
            if (state.landed_state == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND  and
            self.race_end and self.armed):
                print "Drone on the ground"
                self.armed = False
                self.running = False

            # The MAV_LANDED_STATE_TAKEOFF state is not being sent
            # so detect takeoff our self
            elif (self.curr_land_state == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND and
            state.landed_state == mavutil.mavlink.MAV_LANDED_STATE_IN_AIR):
                self.debug("RACE BEGIN!")
                self.record = True
                self.time_start = self.time_ms

            self.curr_land_state = state.landed_state

        """
        @self.on_message('LOCAL_POSITION_NED')
        def local_position_ned_listener(self, name, data):
            if not self.start_position:
                self.start_position = data 

            if not self.time_takeoff and  data.z < self.TAKEOFF_JITTER and self.armed:
                # Normalize
                self.time_takeoff = data.time_boot_ms - self.time_first
                self.debug("Take off detected! {} {} {}".format(self.time_takeoff, self.start_position.z, data.z))


            # TODO move to gate?
            if self.track:
                if self.track.is_out_of_bounds(data.x, data.y, data.z):
                    self.out_of_bounds = True
                #self.info("x={} y={} z={}".format(data.x, data.y, data.z))
                if (self.track.gate_count > self.gate_next and
                    self.track.gates[self.gate_next].detected(data.x, data.y, data.z)):

                    # Keep track from when take off
                    gate_time = data.time_boot_ms - (self.time_takeoff + self.time_first)
                    self.info("Gate {} reached at {}".format(self.gate_next, gate_time))
                    self.gate_times.append(gate_time)

                    # Update if best time
                    # TODO do this after
                    if (not self.track.gates[self.gate_next].time_best or 
                        self.track.gates[self.gate_next].time_best > gate_time):
                        self.info("************** New Gate Record! *********************")
                        self.track.gates[self.gate_next].time_best = gate_time
                        self.track.gates[self.gate_next].time_best_by = self.name 
                    self.gate_next += 1 
                
            self.flight_trajectory.append(data)


    def log_att(self, att):
        self.info("roll={:0.2f} pitch={:0.2f} yaw={:0.2f} thrust={:0.2f}".format(att.body_roll_rate, att.body_pitch_rate, att.body_yaw_rate, att.thrust))

    def set_max_horizontal_velocity(self, v):
        self._master.mav.param_set_send(
            0,
            0,
            "MPC_XY_VEL_MAX",
            v,
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )


    def set_position_target_local_ned(self, x, y, z):

        self._master.mav.set_position_target_local_ned_send(
            0, #time boot not used
            0, #self._master.target_system,
            0, #self._master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, #Positions are relative to the vehicles home position
            0b0000111111111000, # only position
            x, y, z, # x y z in m
            0, 0, 0, # vx, vy, vz
            0, 0, 0, # afx, afy, afz
            0, 0
        )
    @abc.abstractmethod
    def controller(self):
        """ Control the quadrotors trajectory """


    def change_to_offboard_mode(self):
        """Block until we get into offboard mode """
        while self.mode.name != "OFFBOARD" and self.running:
            self._master.set_mode_px4('OFFBOARD', None, None)
            self.debug("Waiting for mode change, current mode={}".format(self.mode.name))
            time.sleep(1)

    def fly(self, name, track, trajectory=None, rate = 0):
        """ Fly the given track with the optional inputs.
        If rate is 0 then send the trajectory as fast as possible

        trajectory:  An array of states in which each state is array containing the time to maintain the state
        the thrust, and target attitutde quaternion
        """
        self.name = name
        self.track = track
        self.input = trajectory
        self.rate = rate
        self.running = True
        
        self.debug("Starting controller thread")
        t = threading.Thread(target=self.controller)
        t.start()
        
        self.debug("Start offboard mode change")
        self.change_to_offboard_mode()
        self.debug("Mode changed to {}".format(self.mode.name))

        self.debug("Arming...")
        self.arm()
        self.debug("Arm complete")

        while self.armed and self.running:
            time.sleep(1)

        t.join()

    def takeoff(self):
        #mask = (0x1000 | 0b110111000011)
        #mask = (0x1000 |0b0000110111000011)
        #mask = (0x1000 | 0b0000110011011011)
        mask = (0x1000 | 0b0000111111111000)
        mask = 0b0000111111111000
        self._master.mav.set_position_target_local_ned_send(
            0, #time boot not used
            0, #self._master.target_system,
            0, #self._master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            mask,
            0, 0, -2, # x y z in m
            0, 0, 0, # vx, vy, vz
            0, 0, 0, # afx, afy, afz
            0, 0
        )


    def arm(self):
        self.armed = True 
        """ Block until the time is set becaues everything is based off this """
        while not self.armed and self.running:# and not self.time_since_armed:      
                time.sleep(0.1)


    def info(self, message):
        logger.info("[World={:.0f} FC={} Boot={} Sim={} Liftoff={}]\t{}".format(self.system_time_us(), self.time_fc_usec, self.time_boot_ms, self.time_ms, self.time_since_takeoff, message))
    def debug(self, message):
        logger.debug("[World={:.0f} FC={} Boot={} Sim={} Liftoff={}]\t{}".format(self.system_time_us(), self.time_fc_usec, self.time_boot_ms, self.time_ms, self.time_since_takeoff, message))

    def system_time_ms(self):
        return time.time() * 1000.0
    def system_time_us(self):
        return time.time() * 1000000.0

    def _sleep(self, start, rate):
        """
        Sleep for specified time based on the starting time
        and the rate in which command should execute
        """
        lapse = time.time() - start
        T = 1/float(self.SET_PT_RATE)
        if lapse < T:
            time.sleep(T - lapse)


    def control_inputs(self):
        if self.armed:
            logger.warn("Can't obtain control inputs while armed")
            return
        return self.flight_data.control_inputs()
            
    def diagnostics(self):
        if self.armed:
            logger.warn("Can't generate diagnostics while armed")
            return
        if self.flight_data:
            self.flight_data.show()

    
class QuadrotorEvolved(QuadrotorPX4):
    REPLAY_RATE = 100.0 #hz

    def __init__(self, *args):
        super(QuadrotorEvolved, self).__init__(*args)

        self.sent_attitude = []


    def fly(self, name, track, input=None, rate = 0):
        # Initialize any different parameters

        self.set_max_horizontal_velocity(30.0)
        self.debug("Playback at {}Hz".format(rate))
        super(QuadrotorEvolved, self).fly(name, track, input, rate)

    def convert_rate_to_time(self, rate):
        """ If rate is 0 then send unlimited """
        if rate == 0:
            return -1
        return 1/rate


    def set_attitude_target(self, att, mask=0b00000111): 
        # According to modules/mavlink/mavlink_receiver.cpp
        # when this message is received the attitude bit is first
        # checked. If it is set quaternion is converted to euler
        # with mavlink_quaternion_to_euler and then published to orb as the
        # vehicle_attitude_setpoint topic

        #self.info("Setting thrust {}".format(att.thrust))
        self._master.mav.set_attitude_target_send(
            0, # time 
            0, # target_system
            0, # target_component
            mask, 
            [att[Q0], att[Q1], att[Q2], att[Q3]],#[0,0,0,0], # att.q,IGNORE ATTITUTDE
            0,#att[ROLL],
            0,#att[PITCH],
            0,#att[YAW],
            att[THRUST]
        )
        """
        if self.armed:
            self.sent_attitude.append(AttitudeTarget(self.system_time_us(), 
                att[ROLL],
                att[PITCH],
                att[YAW],
                att[THRUST],
                [att[Q0], att[Q1], att[Q2], att[Q3]]
                )
                )
        """
    def set_attitude_target2(self, att, mask=0b00000000): 
        # According to modules/mavlink/mavlink_receiver.cpp
        # when this message is received the attitude bit is first
        # checked. If it is set quaternion is converted to euler
        # with mavlink_quaternion_to_euler and then published to orb as the
        # vehicle_attitude_setpoint topic

        #self.info("Setting thrust {}".format(att.thrust))
        self._master.mav.set_attitude_target_send(
            0, # time 
            0, # target_system
            0, # target_component
            mask, 
            att.q,#[0,0,0,0], # att.q,IGNORE ATTITUTDE
            att.body_roll_rate,
            att.body_pitch_rate,
            att.body_yaw_rate,
            att.thrust
        )


    # TODO should we use our own local clock?
    def controller(self):
        """ We record the entire flight to obtain the attitude however we only 
        only play back once armed
        """
        attitudes = deque(self.input)

        # Init
        curr_attitude = attitudes.popleft()
        command_index = 0
        command_send_time =  curr_attitude[0] 
        command_time_start = self.system_time_us()

        while self.running:
            if not self.armed:
                # Keep in OFFBOARD mode
                self.set_attitude_target(curr_attitude, 0b11111111)
                continue
            current_time = self.system_time_us()
            dt = current_time - command_time_start
            if dt < 0.0:
                continue
            if  attitudes and dt >= command_send_time:
                curr_attitude = attitudes.popleft()
                #print "Current thrust=", curr_attitude.thrust
                command_send_time = curr_attitude[0] 
                command_time_start = current_time 
                command_index += 1

            self.set_attitude_target(curr_attitude)

            T = self.convert_rate_to_time(self.REPLAY_RATE)
            time_lapsed_s = (self.system_time_us() - current_time)/1000000.0
            if time_lapsed_s < T:
                time.sleep(T - time_lapsed_s )

        self.debug("Set {} commands".format(command_index))

    """
    def controller2(self):
        self.debug("Thread started")
        (t, roll, pitch, yaw, thrust) = split_input_data(self.input)
        intervals_ms = np.diff(t)

        attitudes = deque(self.input)

        # Init
        curr_attitude = attitudes.popleft()
        command_index = 0
        command_send_time =  intervals_ms[command_index]
        command_time_start = self.time_ms()

        while self.running:
            if not self.armed:
                # Keep in OFFBOARD mode
                self.set_attitude_target(curr_attitude, 0b11111111)
                continue

            dt = self.time_ms() - command_time_start
            if  attitudes and dt >= command_send_time:
                curr_attitude = attitudes.popleft()
                #print "Current thrust=", curr_attitude.thrust
                command_send_time = intervals_ms[command_index]
                command_time_start = self.time_ms()
                command_index += 1

            self.set_attitude_target(curr_attitude)

        self.debug("Set {} commands".format(command_index))
    """


class QuadrotorGuided(QuadrotorPX4):

    def __init__(self, *args):
        super(QuadrotorGuided, self).__init__(*args)

    def controller(self):
        # Way points are local and relative
        gates = self.track.gates
        for i in range(len(gates)):
            if not self.running:
                break

            gate = gates[i]
            #print "Next gate is at ", gate
            self.set_location(gate)

        #self.debug("All waypoints reached, mission thread complete")
        
    def set_location(self, gate):
        """Monitor the progress and set the next point once reached"""

        # While we have not reached our destination keep setting our next set point
        # at the predefined rate
        while self.running:
            start = time.time() # TODO move to _d?
            # TODO add more constraints
            if gate.detected(self.flight_trajectory[-1].x, self.flight_trajectory[-1].y, self.flight_trajectory[-1].z):
                break
            self.set_position_target_local_ned(gate.x, gate.y, gate.z)
            #self._sleep(start, self.SET_PT_RATE)


    def fly(self, name, track, input=None, rate = 0):
        self.set_max_horizontal_velocity(30.0)
        #  The guided quadcopter flys as input only the track 
        super(QuadrotorGuided, self).fly(name, track)


class TrajectoryEvolver(object):
    """ Radius in meters that is accepted to hit the waypoint """
    WAYPOINT_R = 1.0
    SELPB, CXPB, MUTPB, ADDPB, DELPB, MU, NGEN = 0.25, 0.7, 0.4, 0.4, 0.4, 4, 1000
    
    # Kill race if cant take off within this time
    TAKEOFF_TIMEOUT = 10000 #ms

    # Value to be added to fitness for each gate not missed
    GATE_PENALTY = 100000 #ms

    # Dont kill as long as the race time is less than current time + handicap
    GATE_HANDICAP = 10000 #ms

    def __init__(self, px4_home, gazebo_host="127.0.0.1", gazebo_port=11345, px4_host="127.0.0.1", px4_port=14540):

        self.start_time = None
        self.px4_home = px4_home

        log_dir = os.path.join(px4_home, "build_posix_sitl_lpe/tmp/rootfs/fs/microsd/log")
        if not os.path.isdir(log_dir):
            raise argparse.ArgumentTypeError("Log directory {} does not exist".format(log_dir))

        self.px4_log = log_dir

        #logger.info("Starting sim")
        #self.start_sim()
        self.px4_host = px4_host
        self.px4_port = px4_port

        self.px4_connect_string = "{}:{}".format(px4_host, px4_port)
        self.gazebo_host = gazebo_host
        self.gazebo_port = gazebo_port
        #self.gz = GazeboAPI(gazebo_host, gazebo_port)

        # Keep track of each racer identified by their hash
        # including the number of times they have been in the generation
        # how many way points did they reach and their best times
        # where their best log is stored
        self.racers = {}
        self.flight_data = []
        self.flight_data_attitudes = []
        self.flight_data_paths = []
        self.baseline_trajectory = None
        self.best_times = None
        self.vehicle = None
        self.racing = False
        self.first = False
        self.rate = 0

        """ Instance counter for each iteration """
        self.counter = 1

        """ Track all the gate times so we can see improvements """
        self.gate_times = []
        self.track = straight_line_track(num_gates = 2, altitude = -2)
        #self.track = square_track()
        self.gz = GazeboAPI(self.gazebo_host, self.gazebo_port)
        self.gz.listen()
        # have something as the default that will cause a time out if not reached
        """
        t= 0
        for i in range(self.track.gate_count):
            self.track.gates[i].time_best = t
            t += 3000
        """

        logger.info("Race track: {}".format(self.track))
        


        # TODO Add distribution
        # The trajectory is a sequence of states.
        # The vehicle is controlled by the attitude comprising
        # of an array of these fields thus the contains are indexed
        # by the same position they are 
        self.attitude_field_constraints = [
            # we 
            {"min": 1000, "max": 1000000, "resolution": 1000.0 }, # delta time in us, the time in between each state change. min is 1ms, max 1s 
            {"min": 0, "max": 1, "resolution": 1000.0}, #  thrust
            {"min": -20, "max": 20, "resolution": 1000.0}, # Roll in degrees
            {"min": -20, "max": 20, "resolution": 1000.0}, # pitch in degrees
            {"min": -20, "max": 20, "resolution": 1000.0} # Yaw in degrees

        ]


    def _model_reset_callback(self):
        logger.debug("Model reset")


    def timeout_gate(self, vehicle):
        """ At anytime poll to see if  the vehicle hasn't made it the next gate within 
        the time of the best time plus a handicap
        """
        timeout = False


        number_gates_completed = len(vehicle.gate_times)
        if number_gates_completed < self.track.gate_count: # Havnt hit all gates otherwise well end anyway
            last_gate_best_time = self.track.gates[number_gates_completed].time_best
            if vehicle.time_since_takeoff > (last_gate_best_time + self.GATE_HANDICAP):
                timeout = True

        """
        else:
            for i in range(vehicle.track.gate_count):
                if vehicle.gate_times[i] > vehicle.track.gates[i].time_best:
                    is_fastest = False
                    break;
        """
        return timeout

    def timeout_takeoff(self, vehicle):
        return not vehicle.time_takeoff and vehicle.time_ms >  self.TAKEOFF_TIMEOUT

    def race_monitor(self, vehicle):
        logger.debug("Waiting for race to timeout")
        abort = False
        while True:#self.racing:
            if vehicle.track:
                    #(vehicle.track.gate_count > vehicle.gate_next and # There are still gates left
                    #vehicle.time_since_armed  > vehicle.track.gates[vehicle.gate_next].time_best) 
                if self.timeout_gate(vehicle):
                    logger.info("Aborting. Reason: Gate timeout +{} Liftoff {} Best times {} Vehicle times {} ".format(self.GATE_HANDICAP,vehicle.time_since_takeoff, vehicle.track.gates, vehicle.gate_times ))
                    abort = True

                elif (vehicle.track.gate_count == vehicle.gate_next): #completed
                    logger.info("Completed all gates {} {}".format(vehicle.track.gate_count,vehicle.gate_next))
                    abort = True
                elif self.timeout_takeoff(vehicle):
                    logger.info("Takeoff timeout")
                    abort = True
                elif vehicle.out_of_bounds:
                    logger.info("Aircraft out of bounds")
                    abort = True

                if abort:
                    vehicle.running = False #stop all threads
                    vehicle.armed = False
                    break
            time.sleep(1)
            logger.debug("OK t={} Liftoff={} {} {}".format(vehicle.time_ms, vehicle.time_since_takeoff, vehicle.track.gates, vehicle.gate_times))

        self.racing = False
        logger.debug("outside loop")


    def init_search(self):
	# The weights tuple depends on what is returned in the evaluation function
        # We will want to minimize the time
	creator.create("FitnessMin", base.Fitness, weights=(-1.0,-1.0,))
        creator.create("Individual", list, fitness=creator.FitnessMin)

        self.toolbox = base.Toolbox()

        # It has been found that using a heuristic for initialize the entire
        # population will not lead to an optimial solutoin
        #self.toolbox.register("trajectory", self.initial_trajectory)
        self.toolbox.register("individual", tools.initIterate, creator.Individual, self.initial_trajectory_single)
        self.toolbox.register("population", tools.initRepeat, list, self.toolbox.individual)

        # Operator registering
        self.toolbox.register("evaluate",self.evaluate)

        """ For this applicaiton what is the best method to mate?
        We will have trajectories with different number of states, thus if we do 
        mating of a state how do we align?


        """
        self.toolbox.register("mate", self.mate_one_point)
        self.toolbox.register("mutate",self.mutate)
        self.toolbox.register("add", self.mutate_add)
        self.toolbox.register("delete", self.mutate_del)
        self.toolbox.register("select", tools.selBest )


    def initial_trajectory(self):
        """ We can create an individual (1) uniform random (2) by some distriubtion  (3) Using a hueristic as a 
        baseline such as a flight with the PX4 algorithm"""

        # flip a coin, however we need to use a baseline first so the best times are set
        if self.first or np.random.random() < 0.5:
            self.first = False
            t = self.trajectory_from_baseline()
            return t
        else:
        # randoly generating an input will surely cause unstable flight
        # what to do about this
            t = self.generate_random_trajectory()
            return t

    def initial_trajectory_single(self):
        """ Start off with a single command """
        flight_data = []
        flight_data.append(self._generate_random_command())
        return flight_data


    def trajectory_from_baseline(self):
        logger.info("Creating trajectory from guided copter")
        name = "Guided-{}".format(self.counter)
        self.race(name, QuadrotorGuided)
        self.counter += 1
        log = self.parse_log()
        self.rate = self.compute_rate_from_log(log)
        baseline_trajectory =self.convert_flight_log_to_euler_trajectory(log)
        #t = self.baseline_trajectory[:]
        return baseline_trajectory # t


    def constraint_step(self, constraint):
        return (constraint["max"] - constraint["min"]) / constraint["resolution"]

    def continuous_to_discrete(self, value, constraint):
        """Convert a continuous value to a discrete one depending on its constraints"""
        step = self.constraint_step(constraint)
        return math.floor(value/step) * step

    def convert_flight_log_to_euler_trajectory(self, attitude):
        """ Convert a list of AttitudeTarget objects to a 2D array defining the trajectory that will be
        used by the controller to play back.
        This is also what ends up being evolved. Times are converted to deltas
        """
        # A 2D array in which each 
        # row is a command specify how long it should be sent for
        trajectory = []
        for i in range(len(attitude)):
            state = []
            if i+1 < len(attitude):
                dt = attitude[i+1].time_us - attitude[i].time_us
            else:
                # Go as long as it wants on the last one
                dt = 100000000

            state.append(dt)
            #state.append(attitude[i].body_roll_rate)
            #state.append(attitude[i].body_pitch_rate)
            #state.append(attitude[i].body_yaw_rate)
            thrust = attitude[i].thrust
            thrust_discrete = self.continuous_to_discrete(thrust, self.attitude_field_constraints[THRUST])
            state.append(thrust_discrete)
            #state += attitude[i].q
            q = attitude[i].q
            roll, pitch, yaw = quaternion_to_eulerian(q[0], q[1], q[2], q[3])
            roll_discrete = self.continuous_to_discrete(math.degrees(roll), self.attitude_field_constraints[ROLL])
            pitch_discrete = self.continuous_to_discrete(math.degrees(pitch), self.attitude_field_constraints[PITCH])
            yaw_discrete = self.continuous_to_discrete(math.degrees(yaw), self.attitude_field_constraints[YAW])

            state.append(roll_discrete)
            state.append(pitch_discrete)
            state.append(yaw_discrete)

            #print state
            trajectory.append(state)

        return trajectory


    def rand(self, a, b):
        """Helper function to get a random number [a,b)"""
        return (b - a) * np.random.random_sample() + a

    def _generate_random_command(self):
        cmd = []
        for i in range(len(self.attitude_field_constraints)):
            continuous = self.rand(self.attitude_field_constraints[i]["min"], self.attitude_field_constraints[i]["max"])
            discrete = self.continuous_to_discrete(continuous, self.attitude_field_constraints[i])
            cmd.append(discrete)
        return cmd

    def generate_trajectory_from_baseline(self):
        """ Given a baseline we know completes the track, mutate it almost 
        like creating a virtual example"""
        sigma = 0.0001 
        trajectory = []
        if np.random.random() < 0.5:
            for i in range(len(self.baseline_trajectory)):
                state = []
                for j in range(len(self.attitude_field_constraints)):
                    baseline_value = self.baseline_trajectory[i][j]
                    if np.random.random() < 0.1:
                        new_value = np.random.normal(baseline_value, sigma)
                        min = self.attitude_field_constraints[j]["min"]
                        max = self.attitude_field_constraints[j]["max"]
                        if new_value < min:
                            new_value = min
                        elif new_value > max:
                            new_value = max
                        state.append(new_value) 
                    else:
                        state.append(baseline_value)
                trajectory.append(state)
            return trajectory
        else:
            return self.baseline_trajectory

    def generate_random_trajectory(self):
        flight_data = []
        for i in range(np.random.randint(100)): # generate this many random commands
            flight_data.append(self._generate_random_command())
        return flight_data



    def mate_one_point(self, input_A, input_B):
        """ Pick a random point of the smaller and swap the tails"""
        pt = np.random.randint(min(len(input_A), len(input_B)))
        tmp = input_B[pt:]
        input_B[pt:] = input_A[pt:]
        input_A[pt:] = tmp 

        return input_A, input_B
    def mate_trajectory(self, input_A, input_B):
        """ Mate trajectory by swapping an entire trajectory state"""
        # randomly swap some commands
        for i in range(min(len(input_A), len(input_B))):
            #if np.random.random() < 0.5:
            tmp = input_A[i]
            input_A[i] = input_B[i]
            input_B[i] = tmp

        return input_A, input_B

    def mate_trajectory_state_parameter(self, input_A, input_B):
        """Mate individual state parameters"""
        for i in range(min(len(input_A), len(input_B))):

            for j in range(len(self.attitude_field_constraints)):
                #if np.random.random() < 0.5:
                tmp = input_A[i][j]
                input_A[i][j] = input_B[i][j]
                input_B[i][j] = tmp

        return input_A, input_B



    def mutate(self, inputs):
        """ Given a trajectory, mutate the states by first randomly selecting one of the set attitude target
        messages and then randomly selecting one of the inputs and changing its value."""
        
        #Randomly select one of the inputs
        random_cmd_index = np.random.randint(len(inputs))
        random_cmd = inputs[random_cmd_index]

        #Now randomly select an input
        random_input_index = np.random.randint(len(random_cmd))
        current_value = inputs[random_cmd_index][random_input_index] 

        # For a mutation mutate by a step
        constraint = self.attitude_field_constraints[random_input_index]
        step = self.constraint_step(constraint)
        if np.random.random() < 0.5:
            new_input = current_value + step
        else:
            new_input = current_value - step

        # Check new  range in limits
        if new_input < constraint["min"]:
            new_input = constraint["min"]
        elif new_input > constraint["max"]:
            new_input = constraint["max"]




        # TODO  Does it make sense to adjust from the original or 
        # choose it randoml yas we are here?
        #new_input = max * np.random.random_sample() + min 
        inputs[random_cmd_index][random_input_index] = new_input 

        return inputs,

    def mutate_add(self, inputs):
        """Randomly create a new trajectory state"""
        cmd = self._generate_random_command()
        random_cmd_index = np.random.randint(len(inputs))
        #Insert it randomly
        inputs.insert(random_cmd_index, cmd)
        return inputs,

    def mutate_del(self, inputs):
        """ Randomly deleted a trajectory state"""
        if len(inputs) < 2:
            return inputs,
        random_cmd_index = np.random.randint(len(inputs))
        inputs.pop(random_cmd_index)
        return inputs,


    def evaluate_trajectory(self, trajectory):
        """ How close did it make it to its next gate? """
        for t in trajectory:
            pt = [t.x, t.y, t.z]

    def _d(self, pt1, pt2):
        """ Return distance between two points in 3D where each point is an array [x, y, z]"""
        a = np.array(pt1)
        b = np.array(pt2)
        d = np.linalg.norm(a-b)
        return d

    def evaluate_proximity(self, trajectory):
        """Based on the trajectory how close did they get to the gates?"""

        # Set maximums which just calculates the distances 
        # from each gate point by a line, have to get at least better than this
        gate_distances = [0]*self.track.gate_count
        pt = [0, 0, 0]
        for i in range(self.track.gate_count):
            gate = self.track.gates[i]
            gate_pt = [gate.x, gate.y, gate.z]
            d = self._d(pt, gate_pt)
            gate_distances[i] = d
            pt = gate_pt


        # Instead measure how close the trajectory comes
        next_gate = 0
        for i in trajectory:
            pt = [i.x, i.y, i.z]

            gate = self.track.gates[next_gate]
            gate_pt = [gate.x, gate.y, gate.z]
            d = self._d(gate_pt, pt)
            if d < gate_distances[next_gate]:
                gate_distances[next_gate] = d

            if gate.detected(pt[0], pt[1], pt[2]): # Only continue in sequence if we hit the gate
                next_gate += 1

            if next_gate > self.track.gate_count - 1:
                break

        return sum(gate_distances)

    def evaluate(self, inputs):
        _in = copy.deepcopy(inputs)
        name = "E-{}".format(self.counter)
        logger.info("Evaluating {}".format(name))
        trajectory, gate_times = self.race(name, QuadrotorEvolved, input=_in, rate=self.rate)
        latest_log = self.get_last_log() 

        distance_fitness = self.evaluate_proximity(trajectory)

        self.counter += 1
        
        h = hash(str(inputs))
        if not(h in self.racers):
            self.racers[h] = {}
            self.racers[h]["times"] = []
            self.racers[h]["gens"] = 0

        # Update their best time
        if (len(gate_times) > len(self.racers[h]["times"])
        or (len(gate_times) > 0 and  (gate_times[-1] < self.racers[h]["times"][-1]))):
            self.racers[h]["log"] = latest_log.split(self.px4_log)[-1]
            self.racers[h]["times"] = copy.deepcopy(gate_times)
        
        # If they finished the track then we evaluate on their best time
        time_fitness = None
        if len(gate_times) == self.track.gate_count:
            time_fitness = gate_times[-1]
        elif len(gate_times) == 0:
            time_fitness = (self.track.gate_count * self.GATE_PENALTY)
        else:
            # if the aircarft made it to some gates then 
            # take the last reached time then penalize proportional for those not reached
            time_fitness = (gate_times[-1] + ((self.track.gate_count - len(gate_times)) * self.GATE_PENALTY))
        return time_fitness, distance_fitness,

         
    def number_finished_race(self, pop):
        """Given the population how many actually finished the race?"""
        fits = [ind.fitness.values[0] for ind in pop]
        finished = 0
        for f in fits:
            if f < self.GATE_PENALTY:
                finished += 1
        return finished

    def update_racer_hall_of_fame(self, pop):
        """ Update with the number of generations the racer has been through """
        ids = []
        for ind in pop:
            h = hash(str(ind))
            ids.append(h)
            if h in self.racers:
                self.racers[h]["gens"] += 1
            else:
                self.racers[h] = {}
                self.racers[h]["gens"] = 0

        #remove old ones
        keys_to_remove = []
        for key in self.racers:
            if not(key in ids):
                keys_to_remove.append(key)

        for key in keys_to_remove:
            del self.racers[key]


    def print_racer_hall_of_fame(self):
        logger.info("Hall of Fame")
        for key in self.racers:
            log_str = ""
            for param in self.racers[key]:
                log_str += "{} = {} ".format(param, self.racers[key][param])
            logger.info("{} {}".format(key, log_str))

    def start_sim(self):
        os.environ["HEADLESS"] = "1"
        #call(["make", "posix_sitl_lpe", "gazebo"], cwd=self.px4_home, shell=True)
        print self.px4_home
        cmd = "make posix_sitl_lpe gazebo"
        p = Popen(cmd, cwd=self.px4_home, shell=True)
        time.sleep(10)

    def start(self):

        signal.signal(signal.SIGINT, self.signal_handler)
        np.random.seed(1)

        # We need to fly guided at least once so we can set up what the
        # gate times are
        self.race("baseline", QuadrotorGuided)


        logbook = tools.Logbook()
        logbook.header = "gen", "time_fitness", "proximity_fitness"
        logbook.chapters["time_fitness"].header = "min", "avg", "max", "std"
        logbook.chapters["proximity_fitness"].header = "min", "avg", "max", "std"

        stats_fit_time = tools.Statistics(key=lambda ind: ind.fitness.values[0])
        stats_fit_proximity = tools.Statistics(key=lambda ind: ind.fitness.values[1])
        mstats = tools.MultiStatistics(time_fitness=stats_fit_time, proximity_fitness=stats_fit_proximity)

        mstats.register("avg", np.mean)
        mstats.register("std", np.std)
        mstats.register("min", np.min)
        mstats.register("max", np.max)

        self.init_search()

        pop = self.toolbox.population(n=self.MU)


        logger.info("Evalutating initial population")
        # Evaluate every individuals
        fitnesses = self.toolbox.map(self.toolbox.evaluate, pop)
        for ind, fit in zip(pop, fitnesses):
            ind.fitness.values = fit

        self.update_racer_hall_of_fame(pop)
        self.print_racer_hall_of_fame()


        gen = 1
        while gen <= self.NGEN: # and (logbook[-1]["max"][0] != 0.0 or logbook[-1]["max"][1] != 0.0):
            logger.info("###############################################")
            logger.info("                  GEN-{}".format(gen))
            logger.info("###############################################")


            # Next we perform cross over and mutation. We select from the population
            # those that will be combined to form new individuals

            # Select the next generation individuals, only 
            # select the best to mate
            offspring = self.toolbox.select(pop, int(len(pop) * self.SELPB))
            # Clone the selected individuals
            offspring = list(map(self.toolbox.clone, offspring))
            logger.info("Number of parents %s" % len(offspring))
            # Apply crossover and mutation on the offspring
            offspring = offspring * int(1/ self.SELPB)
            # Use technique from "Evolving Mobile Robots in Simulated and Real Environments"
            # and dont do cross over
            """
            for child1, child2 in zip(offspring[::2], offspring[1::2]):
                if np.random.random() < self.CXPB:

                    logger.info("Crossover")
                    self.toolbox.mate(child1, child2)
                    del child1.fitness.values
                    del child2.fitness.values
            """


            logger.info("Number of offsprint after mating %s" % len(offspring))
            for mutant in offspring:
                h = hash(str(mutant))
                if np.random.random() < self.MUTPB:
                    logger.info("{} Mutate Modify".format(h))
                    self.toolbox.mutate(mutant)
                    del mutant.fitness.values
                if np.random.random() < self.ADDPB:
                    logger.info("{} Mutate Add".format(h))
                    self.toolbox.add(mutant)
                    del mutant.fitness.values
                if np.random.random() < self.DELPB:
                    logger.info("{} Mutate Del".format(h))
                    self.toolbox.delete(mutant) 
                    del mutant.fitness.values

            # Evaluate the individuals with an invalid fitness
            logger.info("Evalulating the offspring...")
            invalid_ind = [ind for ind in offspring if not ind.fitness.valid]
            fitnesses = map(self.toolbox.evaluate, invalid_ind)
            for ind, fit in zip(invalid_ind, fitnesses):
                ind.fitness.values = fit

            logger.info("  Evaluated %i individuals" % len(invalid_ind))

           
            logger.info("Population size %s offspring size %s" % (len(pop), len(offspring)))
            # combine the existing population with the offspring
            # and take the best ones 
            """
            new_offspring = []
            for i in offspring:
                if not(i in pop):
                    new_offspring.append(i)
            pop[:] = self.toolbox.select(pop + new_offspring, self.MU)
            """
            pop[:] = offspring
            gen += 1

 # Gather all the fitnesses in one list and print the stats
            fits_time = [ind.fitness.values[0] for ind in pop]
            fits_proximity = [ind.fitness.values[1] for ind in pop]

            self.update_racer_hall_of_fame(pop)
            self.print_racer_hall_of_fame()

            record = mstats.compile(pop)
            logbook.record(gen=gen, **record)
            print logbook

            # Generate stats
            """
	    length = len(pop)
	    mean = sum(fits) / length
	    sum2 = sum(x*x for x in fits)
	    std = abs(sum2 / length - mean**2)**0.5


            logger.info("  Finished Race %s" % self.number_finished_race(pop))
	    logger.info("  Min %s" % min(fits))
	    logger.info("  Max %s" % max(fits))
	    logger.info("  Avg %s" % mean)
	    logger.info("  Std %s" % std)
            """

            """
            if self.check_log_flag():
                data = FlightAnalysis()
                data.plot_input(self.flight_data)
                data.save()
            """
            #logger.info("Restarting sim")
            #self.start_sim()

        return pop



    def repeat(self):
        self.race("baseline", QuadrotorGuided)
        #self.race("baseline2", QuadrotorGuided)
        #self.race("baseline3", QuadrotorGuided)
        log = self.parse_log()
        rate = self.compute_rate_from_log(log)
        logger.info("Data appears to be logged at {} Hz".format(rate))
        baseline_logged_att_sp = self.convert_flight_log_to_euler_trajectory(log)

        #self.race("shadow-logged", QuadrotorEvolved, input=baseline_logged_att_sp, rate = rate)

        #self.flight_data.append(FlightData("shadow-sent", None, None, self.vehicle.sent_attitude ))

        #self.race("shadow2-logged", QuadrotorEvolved, input=baseline_logged_att_sp, rate = rate)
        data = FlightAnalysis(self.track.gates, self.flight_data)
        data.plot_input(self.flight_data)
        data.plot_3D_path()
        data.save()

    def convert_trajectory_euler_to_quaternion(self, trajectory):
        
        for i in trajectory:
            roll, pitch, yaw = i[-3:] # attitude is last
            w, x, y, z = eulerian_to_quaternion(math.radians(roll), math.radians(pitch), math.radians(yaw))
            i[-3:] = [w, x, y, z]

        return trajectory


    def race(self, name, vehicle_class, input=None, rate = 0):
        """ Perform once race with the specified vehicle  following the given trajectory"""
        self.racing = True

        #logger.info("###########################################################")
        logger.info("                RACE STARTING: {}                          ".format(name))
        #logger.info("###########################################################")


        self.vehicle = connect(self.px4_connect_string, wait_ready=True, vehicle_class=vehicle_class, status_printer=None, heartbeat_timeout=60)
        self.gz.vehicle = self.vehicle

        # Monitor the race and abort if things go wrong
        t = threading.Thread(target=self.race_monitor, args=(self.vehicle,))
        t.start()


        if input:
            input = self.convert_trajectory_euler_to_quaternion(input)
        self.vehicle.fly(name, self.track, input, rate = rate)
        while self.racing:
            time.sleep(1)
        logger.debug("Joining...")
        t.join()
        self.gz.vehicle = None
        self.vehicle.close()

        # Collect the flight data from the race
        """
        try:
            logged_att_sp = self.parse_log()
            #for i in logged_att_sp[:2]:
            #    print i
            self.flight_data.append(FlightData(name, self.vehicle.gate_times, self.vehicle.flight_trajectory, logged_att_sp))
        except Exception as e:
            logger.error(e)
        """

        # Update the best times
        self.track = self.vehicle.track
        logger.info("Race stats {}".format(self.track))

        #Reset the model so we can start again
        #GazeboAPI(self.gazebo_host, self.gazebo_port).reset_model(self._model_reset_callback)
        self.gz.reset()


        return self.vehicle.flight_trajectory, self.vehicle.gate_times


    def get_last_log(self):
        path = "{}/*/*.ulg".format(self.px4_log) 
#"/home/wil/workspace/buflightdev/PX4/build_posix_sitl_lpe/tmp/rootfs/fs/microsd/log/2017-08-29/*.ulg"
        list_of_files = glob.glob(path)
        return max(list_of_files, key=os.path.getctime)

    def compute_rate_from_log(self, data):
        return float( len(data) ) / ( (data[-1].time_boot_ms/1000.0) - (data[0].time_boot_ms/1000.0) ) 

    def parse_log(self):
        """ Return a list of AttitudeTarget times are in microseconds. 
        The data logged begins when armed and ends when disarmed
        """
        latest_log = self.get_last_log() 
        logger.info("Reading log {}".format(latest_log))
        
        msg_filter = ['vehicle_attitude_setpoint']
        ulog = ULog(latest_log, msg_filter)
        data = ulog.data_list

        attitude_sp = []


        for d in data:

            # use same field order as in the log, except for the timestamp
            # Timestamp in log is in microseconds
            start_time = None
            data_keys = [f.field_name for f in d.field_data]
            for i in range(len(d.data['timestamp'])):
                if not start_time:
                    start_time = d.data["timestamp"][i]

                #for k in range(len(data_keys)):
                    #d.data[data_keys[k]][i]
                """
                att = {"time_boot_ms": d.data["timestamp"][i] - start_time, 
                       "body_roll_rate": d.data["roll_body"][i], 
                       "body_pitch_rate": d.data["pitch_body"][i],
                       "body_yaw_rate": d.data["yaw_body"][i],
                       "thrust": d.data["thrust"][i]
                       }
                """
                t = (d.data["timestamp"][i] - start_time)/1000.0 #convert to milliseconds
                attitude_sp.append(AttitudeTarget(
                                          d.data["timestamp"][i], #t, 
                                          d.data["roll_body"][i],
                                          d.data["pitch_body"][i],
                                          d.data["yaw_body"][i], 
                                          d.data["thrust"][i],
                                          [d.data["q_d[0]"][i], d.data["q_d[1]"][i], d.data["q_d[2]"][i], d.data["q_d[3]"][i]]
                                        ))
        return attitude_sp

    def check_log_flag(self):
        flag = "log.flag"
        os.path.isfile(flag)


    def signal_handler(self, signal, frame):
        logger.info('You pressed Ctrl+C! Aborting')
        #Reset the model so we can start again
        #GazeboAPI(self.gazebo_host, self.gazebo_port).reset_model(self._model_reset_callback)
        self.gz.reset()
        self.gz.shutdown()
        self.racing = False
        self.vehicle.running = False #stop all threads
        self.vehicle.armed = False
        self.vehicle.close()
        sys.exit(0)

class AttitudeTarget:
    """This model provides a common interface to access same atittude data 
    whether obtained directly from MAVLink messages or logs
    """
    def __init__(self, time_us, body_roll_rate, body_pitch_rate, body_yaw_rate, thrust, q):
        self.time_boot_ms = time_us/1000.0
        self.body_roll_rate = body_roll_rate
        self.body_pitch_rate = body_pitch_rate
        self.body_yaw_rate = body_yaw_rate
        self.thrust = thrust
        self.q = q
        self.time_us = time_us

    def __str__(self):
        return "ms={} roll={} pitch={} yaw={} thrust={} q={}".format(self.time_boot_ms, self.body_roll_rate, self.body_pitch_rate, self.body_yaw_rate, self.thrust, self.q) 

    def __repr__(self):
        return self.__str__()

class FlightData:
    """
    Data for one flight
    """
    def __init__(self, name, gate_times, positions, attitudes):
        self.name = name
        """ Array of times reaching each gate """
        self.gate_times = gate_times

        """ An array of LOCAL_POSITION_NED messages """
        self.positions =  positions

        """ An array of Attitude SPs """
        self.attitudes = attitudes



def check_directory(value):
    if not os.path.isdir(value):
        raise argparse.ArgumentTypeError("PX4 home directory {} does not exist".format(value))


    return value

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("px4", help="Home directory of PX4 firmware", type=check_directory)
    args = parser.parse_args()

    init_logging()
    #log_path = "/home/wil/workspace/buflightdev/PX4/build_posix_sitl_lpe/tmp/rootfs/fs/microsd/log"
    e = TrajectoryEvolver(args.px4)
    e.start()
    e.gz.shutdown()
    #e.repeat()




