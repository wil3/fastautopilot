"""
Minimize time to fly path
"""

import logging.config
from dronekit import connect, Command, LocationGlobal, Vehicle, VehicleMode
from pymavlink import mavutil
import time, sys, argparse, math, os, yaml
import numpy as np
from tracker import *
import threading
from collections import deque

from sim.gazebo.gazeboapi import GazeboAPI 
import abc 

from track import * 
from pyulog.core import ULog

import glob
import signal


from deap import algorithms
from deap import base
from deap import creator
from deap import tools

DT = 0
ROLL = 1
PITCH = 2
YAW = 3
THRUST = 4
Q0 = 5
Q1 = 6
Q2 = 7
Q3 = 8

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
                self.info("Armed at {}".format(self.time_armed))

            if self.time_armed:
                self.time_since_armed = self.time_ms - self.time_armed 

            if self.time_takeoff:
                self.time_since_takeoff = self.time_ms - self.time_takeoff
                

        @self.on_message('HEARTBEAT')
        def heartbeat(self, name, beat):
            self.info("HEARTBEAT")
        @self.on_message('ATTITUDE')
        def attitude_listener(self, name, attitude):
            #print attitude
            pass

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
                self.info("RACE BEGIN!")
                self.record = True
                self.time_start = self.time_ms

            self.curr_land_state = state.landed_state

        @self.on_message('LOCAL_POSITION_NED')
        def local_position_ned_listener(self, name, data):
            if not self.start_position:
                self.start_position = data 

            if not self.time_takeoff and  data.z < self.TAKEOFF_JITTER and self.armed:
                # Normalize
                self.time_takeoff = data.time_boot_ms - self.time_first
                self.info("Take off detected! {} {} {}".format(self.time_takeoff, self.start_position.z, data.z))


            # TODO move to gate?
            if self.track:
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
            self.info("Waiting for mode change, current mode={}".format(self.mode.name))
            time.sleep(1)

    def fly(self, name, track, trajectory=None, rate = 0):
        """ Fly the given track with the optional inputs.
        If rate is 0 then send the trajectory as fast as possible"""
        self.name = name
        self.track = track
        self.input = trajectory
        self.rate = rate
        self.running = True
        
        self.info("Starting controller thread")
        t = threading.Thread(target=self.controller)
        t.start()
        
        self.info("Start offboard mode change")
        self.change_to_offboard_mode()
        self.info("Mode changed to {}".format(self.mode.name))

        self.info("Arming...")
        self.arm()
        self.info("Arm complete")

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

    def start_sim_time_callback(self, sim_time):
        print "Start Sim Time", sim_time.sec
        
    def end_sim_time_callback(self, sim_time):
        print "End Sim Time", sim_time.sec

    def arm(self):
        self.armed = True 
        """ Block until the time is set becaues everything is based off this """
        while not self.armed and self.running:# and not self.time_since_armed:      
                time.sleep(0.1)


    def info(self, message):
        logger.info("[World={:.0f} FC={} Boot={} Sim={} Liftoff={}]\t{}".format(self.system_time_us(), self.time_fc_usec, self.time_boot_ms, self.time_ms, self.time_since_takeoff, message))

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

    def __init__(self, *args):
        super(QuadrotorEvolved, self).__init__(*args)

        self.sent_attitude = []


    def fly(self, name, track, input=None, rate = 0):
        # Initialize any different parameters

        self.set_max_horizontal_velocity(30.0)
        self.info("Playback at {}Hz".format(rate))
        super(QuadrotorEvolved, self).fly(name, track, input, rate)

    def convert_rate_to_time(self, rate):
        """ If rate is 0 then send unlimited """
        if rate == 0:
            return -1
        return 1/rate


    def set_attitude_target(self, att, mask=0b00000000): 
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
            att[ROLL],
            att[PITCH],
            att[YAW],
            att[THRUST]
        )
        if self.armed:
            self.sent_attitude.append(AttitudeTarget(self.system_time_us(), 
                att[ROLL],
                att[PITCH],
                att[YAW],
                att[THRUST],
                [att[Q0], att[Q1], att[Q2], att[Q3]]
                )
                )
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

            T = self.convert_rate_to_time(self.rate)
            time_lapsed_s = (self.system_time_us() - current_time)/1000000.0
            if time_lapsed_s < T:
                time.sleep(T - time_lapsed_s )

        self.info("Set {} commands".format(command_index))

    def controller2(self):
        self.info("Thread started")
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

        self.info("Set {} commands".format(command_index))


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
            print "Next gate is at ", gate
            self.set_location(gate)

        self.info("All waypoints reached, mission thread complete")
        
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
            self._sleep(start, self.SET_PT_RATE)


    def fly(self, name, track, input=None, rate = 0):
        self.set_max_horizontal_velocity(1.0)
        #  The guided quadcopter flys as input only the track 
        super(QuadrotorGuided, self).fly(name, track)


class TrajectoryEvolver(object):
    """ Radius in meters that is accepted to hit the waypoint """
    WAYPOINT_R = 1.0
    CXPB, MUTPB, ADDPB, DELPB, MU, NGEN = 0.7, 0.4, 0.4, 0.4, 100, 10
    
    TAKEOFF_TIMEOUT = 10000 #ms
    GATE_HANDICAP = 10000 #ms

    def __init__(self, logpath, gazebo_host="127.0.0.1", gazebo_port=11345, px4_host="127.0.0.1", px4_port=14540):
        self.logpath = logpath
        self.px4_connect_string = "{}:{}".format(px4_host, px4_port)
        self.gazebo_host = gazebo_host
        self.gazebo_port = gazebo_port
        self.gz = GazeboAPI(gazebo_host, gazebo_port)

        self.flight_data = []
        self.flight_data_attitudes = []
        self.flight_data_paths = []
        self.baseline_trajectory = None
        self.best_times = None
        self.vehicle = None
        self.racing = False

        """ Instance counter for each iteration """
        self.counter = 1

        """ Track all the gate times so we can see improvements """
        self.gate_times = []
#        self.track = straight_line_track(num_gates = 1, altitude = -2)
        self.track = square_track()

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
            {"min": 0, "max": 1000}, # delta time, the time in between each state change 
            {"min": -(1.0 * 3.1415), "max": 1.0 * 3.1415}, # body roll rate
            {"min": -(1.0 * 3.1415), "max": 1.0 * 3.1415}, # pitch
            {"min": -(1.0 * 3.1415), "max": 1.0 * 3.1415}, # yaw 
            {"min": 0, "max": 1}, #  thrust
            {"min": 0, "max": 1}, #  q0 
            {"min": 0, "max": 1}, #  q1 
            {"min": 0, "max": 1}, #  q2 
            {"min": 0, "max": 1}, #  q3 

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
        logger.info("Waiting for race to timeout")
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

                if abort:
                    vehicle.running = False #stop all threads
                    vehicle.armed = False
                    break
            time.sleep(1)
            logger.info("OK t={} Liftoff={} {} {}".format(vehicle.time_ms, vehicle.time_since_takeoff, vehicle.track.gates, vehicle.gate_times))

        self.racing = False
        logger.info("outside loop")


    def init_search(self):
	# The weights tuple depends on what is returned in the evaluation function
        # We will want to minimize the time
	creator.create("FitnessMin", base.Fitness, weights=(-1.0,))
        creator.create("Individual", list, fitness=creator.FitnessMin)

        self.toolbox = base.Toolbox()

        self.toolbox.register("trajectory", self.trajectory_from_baseline)
        self.toolbox.register("individual", tools.initIterate, creator.Individual, self.toolbox.trajectory)
        self.toolbox.register("population", tools.initRepeat, list, self.toolbox.individual)

        # Operator registering
        self.toolbox.register("evaluate",self.evaluate)
        self.toolbox.register("mate", self.mate)
        self.toolbox.register("mutate",self.mutate)
        self.toolbox.register("addFilter", self.mutate_add)
        self.toolbox.register("delFilter", self.mutate_del)
        self.toolbox.register("select", tools.selBest )

    def trajectory_from_baseline(self):
        t = self.baseline_trajectory[:]
        return t

    def convert_parsed_log_to_trajectory(self, attitude):
        """ Convert a list of AttitudeTarget objects to a 2D array defining the trajectory.
        Times are converted to deltas
        """
        # A 2D array in which each 
        # row is a command specify how long it should be sent for
        input = []
        for i in range(len(attitude)):
            cmd = []
            if i+1 < len(attitude):
                dt = attitude[i+1].time_us - attitude[i].time_us
            else:
                # Go as long as it wants on the last one
                dt = 100000

            cmd.append(dt)
            cmd.append(attitude[i].body_roll_rate)
            cmd.append(attitude[i].body_pitch_rate)
            cmd.append(attitude[i].body_yaw_rate)
            cmd.append(attitude[i].thrust)
            cmd += attitude[i].q

            input.append(cmd)

        return input

        pass

    def _generate_random_command(self):
        
        cmd = []
        for i in range(len(self.attitude_field_constraints)):
            cmd.append(self.attitude_field_constraints[i]["max"] * np.random.random_sample() + self.attitude_field_constraints[i]["min"])
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




    def mate_commands(self, input_A, input_B):
        # randomly swap some commands
        for i in range(min(len(input_A), len(input_B))):
            if np.random.random() < 0.5:
                tmp = input_A[i]
                input_A[i] = input_B[i]
                input_B[i] = tmp

        return input_A, input_B

    def mate_inputs(self, input_A, input_B):
        for i in range(min(len(input_A), len(input_B))):

            for j in range(len(self.attitude_field_constraints)):
                if np.random.random() < 0.5:
                    tmp = input_A[i][j]
                    input_A[i][j] = input_B[i][j]
                    input_B[i][j] = tmp

        return input_A, input_B


    def mate(self, input_A, input_B):

        # There are two ways we can mate
        # by command or input
        #
        input_A, inputB = self.mate_commands(input_A, input_B)
        input_A, inputB = self.mate_inputs(input_A, input_B)
        
        
        return input_A, input_B

    def mutate(self, inputs):
        """ Given a trajectory, mutate the states by first randomly selecting one of the set attitude target
        messages and then randomly selecting one of the inputs and changing its value."""
        
        #Randomly select one of the inputs
        random_cmd_index = np.random.randint(len(inputs))
        random_cmd = inputs[random_cmd_index]

        #Now randomly select an input
        random_input_index = np.random.randint(len(random_cmd))
        max = self.attitude_field_constraints[random_input_index]["max"]
        min = self.attitude_field_constraints[random_input_index]["min"]

        # TODO  Does it make sense to adjust from the original or 
        # choose it randoml yas we are here?
        new_input = max * np.random.random_sample() + min 
        random_cmd[random_input_index] = new_input
        

        return inputs,

    def mutate_add(self, inputs):
        cmd = self._generate_random_command()
        random_cmd_index = np.random.randint(len(inputs))
        #Insert it randomly
        inputs.insert(random_cmd_index, cmd)
        return inputs,

    def mutate_del(self, inputs):
        random_cmd_index = np.random.randint(len(inputs))
        inputs.pop(random_cmd_index)
        return inputs,


    def evaluate_trajectory(self, trajectory):
        """ How close did it make it to its next gate? """
        for t in trajectory:
            pt = [t.x, t.y, t.z]

    def _d(self, pt1, pt2):
        """
        Return distance between two points in 3D
        """
        a = np.array(pt1)
        b = np.array(pt2)
        d = np.linalg.norm(a-b)
        return d


    def evaluate(self, inputs):
        name = "E-{}".format(self.counter)
        logger.info("Evaluating {}".format(name))
        trajectory, gate_times = self.race(name, QuadrotorEvolved, input=inputs, rate=self.rate)

        self.counter += 1
        
        GATE_PENALTY = 10000
        
        # If they finished the track then we evaluate on their best time
        if len(gate_times) == self.track.gate_count:
            return gate_times[-1],
        elif len(gate_times) == 0:
            return (self.track.gate_count * GATE_PENALTY),
        else:
            return (gate_times[-1] + (self.track.gate_count - len(gate_times)) * GATE_PENALTY),
            

        return gate_times[-1],

         



    def start(self):
        signal.signal(signal.SIGINT, self.signal_handler)
        np.random.seed(1)
        
        self.baseline_path, self.best_times = self.race("baseline", QuadrotorGuided)
        log = self.parse_log()
        self.rate = self.compute_rate_from_log(log)
        self.baseline_trajectory = self.convert_parsed_log_to_trajectory(log)

        self.init_search()

        pop = self.toolbox.population(n=20)
        """
	hof = tools.ParetoFront()
        
        stats = tools.Statistics(lambda ind: ind.fitness.values)
        stats.register("min", min)
        stats.register("max", max)
        
        logbook = tools.Logbook()
        logbook.header = "gen", "evals", "min", "[good,evil,len]", "best"
        

        hof.update(pop)
        record = stats.compile(pop)
        logbook.record(gen=0, evals=len(pop), **record)
        """

        # Evaluate every individuals
        fitnesses = self.toolbox.map(self.toolbox.evaluate, pop)
        for ind, fit in zip(pop, fitnesses):
            ind.fitness.values = fit


        gen = 1
        while gen <= self.NGEN: # and (logbook[-1]["max"][0] != 0.0 or logbook[-1]["max"][1] != 0.0):
            # Select the next generation individuals
            offspring = self.toolbox.select(pop, len(pop))
            # Clone the selected individuals
            offspring = list(map(self.toolbox.clone, offspring))
            # Apply crossover and mutation on the offspring
            for child1, child2 in zip(offspring[::2], offspring[1::2]):
                if np.random.random() < self.CXPB:
                    self.toolbox.mate(child1, child2)
                    del child1.fitness.values
                    del child2.fitness.values

            for mutant in offspring:
                if np.random.random() < self.MUTPB:
                    self.toolbox.mutate(mutant)
                    del mutant.fitness.values
                if np.random.random() < self.ADDPB:
                    self.toolbox.addFilter(mutant)
                    del mutant.fitness.values
                if np.random.random() < self.DELPB:
                    self.toolbox.delFilter(mutant) 
                    del mutant.fitness.values

            # Evaluate the individuals with an invalid fitness
            invalid_ind = [ind for ind in offspring if not ind.fitness.valid]
            fitnesses = map(self.toolbox.evaluate, invalid_ind)
            for ind, fit in zip(invalid_ind, fitnesses):
                ind.fitness.values = fit
            
            b = tools.selBest(pop, k=1)[0]
           
            # Select the next generation population
            pop = self.toolbox.select(pop + offspring, self.MU)
            gen += 1


            if self.check_log_flag():
                data = FlightAnalysis()
                data.plot_input(self.flight_data)
                data.show()

        return pop



    def repeat(self):
        self.race("baseline", QuadrotorGuided)

        log = self.parse_log()
        rate = self.compute_rate_from_log(log)
        logger.info("Data appears to be logged at {} Hz".format(rate))
        baseline_logged_att_sp = self.convert_parsed_log_to_trajectory(log)

        self.race("shadow-logged", QuadrotorEvolved, input=baseline_logged_att_sp, rate = rate)

        #self.flight_data.append(FlightData("shadow-sent", None, None, self.vehicle.sent_attitude ))

        #self.race("shadow2-logged", QuadrotorEvolved, input=baseline_logged_att_sp, rate = rate)

        data = FlightAnalysis(self.track.gates, self.flight_data)
        data.plot_input(self.flight_data)
        data.plot_3D_path()
        data.show()


    def race(self, name, vehicle_class, input=None, rate = 0):
        """ Perform once race with the specified vehicle  following the given trajectory"""
        self.racing = True

        logger.info("###########################################################")
        logger.info("                RACE STARTING: {}                          ".format(name))
        logger.info("###########################################################")


        self.vehicle = connect(self.px4_connect_string, wait_ready=True, vehicle_class=vehicle_class, status_printer=None)

        # Monitor the race and abort if things go wrong
        t = threading.Thread(target=self.race_monitor, args=(self.vehicle,))
        t.start()

        self.vehicle.fly(name, self.track, input, rate = rate)
        while self.racing:
            time.sleep(1)
        logger.info("Joining...")
        t.join()
        self.vehicle.close()

        # Collect the flight data from the race
        try:
            logged_att_sp = self.parse_log()
            for i in logged_att_sp[:2]:
                print i
            self.flight_data.append(FlightData(name, self.vehicle.gate_times, self.vehicle.flight_trajectory, logged_att_sp))
        except Exception as e:
            logger.error(e)

        # Update the best times
        self.track = self.vehicle.track
        logger.info("Race stats {}".format(self.track))

        #Reset the model so we can start again
        GazeboAPI(self.gazebo_host, self.gazebo_port).reset_model(self._model_reset_callback)


        return self.vehicle.flight_trajectory, self.vehicle.gate_times


    def get_last_log(self):
        path = "{}/*/*.ulg".format(self.logpath) 
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
        GazeboAPI(self.gazebo_host, self.gazebo_port).reset_model(self._model_reset_callback)
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




if __name__ == "__main__":
    init_logging()
    log_path = "/home/wil/workspace/buflightdev/PX4/build_posix_sitl_lpe/tmp/rootfs/fs/microsd/log"
    e = TrajectoryEvolver(log_path)
    #e.start()
    e.repeat()



