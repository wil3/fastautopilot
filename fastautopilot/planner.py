
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

from track import straight_line_track, SimplePointGate
from pyulog.core import ULog

import glob

from deap import algorithms
from deap import base
from deap import creator
from deap import tools

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


class PX4Quadrotor(Vehicle):
    """
    Copter is contolled using SET_POSITION_TARGET_LOCAL_NED and SET_ATTITUDE_TARGET 

    https://github.com/PX4/Firmware/issues/1288

    For offboard,
    http://discuss.px4.io/t/offboard-automatic-takeoff-landing-using-mav-cmd-land-takeoff-local/1333/3

    If we are not always sending target location message it switches to RTL

    """

    """ Rate to check if waypoint is hit in hertz """
    SET_PT_RATE = 20

    INIT_ALTITUDE = -2

    """
    MUST PUBLISH FASTER THAN  2HZ TO STAY IN OFFBOARD
    https://dev.px4.io/en/ros/mavros_offboard.html """
    OFFBOARD_MODE_RATE = 4

    def __init__(self, *args):
        super(PX4Quadrotor, self).__init__(*args)

        # Track the trajectory
        self.flight_trajectory = []
        # Track the inputs
        self.flight_attitudes = []

        self.current_time = None

        self.start_position = None

        self.time_since_takeoff = None

        self.time_takeoff = None
        
        self.time_since_armed = None

        self.time_armed = None

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
            loggerPX4.info(text)
        @self.on_message('SYSTEM_TIME')
        def system_time_listener(self, name, t):
            self.current_time = t.time_unix_usec
            self.time_boot_ms = t.time_boot_ms
            if self.armed and not self.time_armed:
                self.time_armed = t.time_boot_ms
                self.info("Armed at {}".format(self.time_armed))

            if self.armed and self.time_armed:
                self.time_since_armed = self.time_boot_ms - self.time_armed 

            if self.time_takeoff:
                self.time_since_takeoff = t.time_boot_ms - self.time_takeoff

            #if self.time_first_sample:
            #    self.time_since_first_sample

        @self.on_message('ATTITUDE_TARGET')
        def attitude_target_listener(self, name, target):
            #sys.stdout.write('.')

            #print "{} {}".format(target.time_boot_ms, target.thrust)
            #if self.armed and self.time_since_armed:# and self.record:
            #target.time_boot_ms = self.time_since_armed

            if not self.time_first_sample:
                print "first sample!"
                self.time_first_sample = target.time_boot_ms

            #if self.time_takeoff:
            self.time_last_attitude_sample = target.time_boot_ms - self.time_first_sample
            target.time_boot_ms = self.time_last_attitude_sample 
            #self.flight_attitudes.append(target)
            #self.log_att(target)
            #print target



        """
        @self.on_message('SET_ATTITUDE_TARGET')                
        def set_attitude_target_listener(self, name, att):
            print att
        """

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
                self.time_start = self.current_time

            self.curr_land_state = state.landed_state

        @self.on_message('LOCAL_POSITION_NED')
        def local_position_ned_listener(self, name, data):
            #print "Z=", data.z 
            #pass
            if not self.start_position:
                self.start_position = data 
            #abs(self.start_position.z - data.z)
            if not self.time_takeoff and  data.z < -0.1 and self.armed:
                self.info("TAKE OFF DETECTED! {} {} {}".format(self.time_takeoff, self.start_position.z, data.z))
                self.time_takeoff = data.time_boot_ms

            self.flight_trajectory.append(data)

            # TODO move to gate?
            if self.track:
                
                if (self.track.gate_count > self.gate_next and
                    self.track.gates[self.gate_next].detected(data.x, data.y, data.z)):

                    gate_time = data.time_boot_ms - self.time_takeoff#self.time_first_sample#self.time_since_armed

                    self.info("Gate {} reached at {}".format(self.gate_next, gate_time))#self.time_since_armed))
                   
                    self.gate_times.append(gate_time)

                    # Update if best time
                    if not self.track.gates[self.gate_next].time_best or self.track.gates[self.gate_next].time_best > gate_time:
                        self.track.gates[self.gate_next].time_best = gate_time
                    self.gate_next += 1 
                


    def log_att(self, att):
        self.info("roll={:0.2f} pitch={:0.2f} yaw={:0.2f} thrust={:0.2f}".format(att.body_roll_rate, att.body_pitch_rate, att.body_yaw_rate, att.thrust))
    # FIXME I think actual commands need to be sent, not this 
    # one to keep in offboard mode, for safety
    def keep_in_offboard_mode(self):
        while self.running:
            self._master.set_mode_px4('OFFBOARD', None, None)
            time.sleep(1/float(self.OFFBOARD_MODE_RATE))
            if not(self.mode == self.curr_mode):
                print "Changed modes ",self.mode
                self.curr_mode = self.mode

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

    def fly(self, track, input=None):
        """ Fly the given track with the optional inputs """
        pass

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
        while not self.armed:# and not self.time_since_armed:      
                time.sleep(0.1)


    def info(self, message):
        logger.info("[{}]\t{}".format(self.time_last_attitude_sample, message))

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

    
class EvolvedQuadrotor(PX4Quadrotor):

    def __init__(self, *args):
        super(EvolvedQuadrotor, self).__init__(*args)


    def fly(self, track, input=None):
        self.track = track

        self.input = input
        self.running = True
        self.set_max_horizontal_velocity(30.0)

        t = threading.Thread(target=self.controller)#, args=(self.waypoint_reached_callback,))
        t.start()
        self.threads.append(t)


        while self.mode.name != "OFFBOARD":
            self._master.set_mode_px4('OFFBOARD', None, None)
            self.info("Waiting for mode change, current mode={}".format(self.mode.name))
            time.sleep(1)

        self.info("Mode={}".format(self.mode.name))
        self.info("Arming...")
        self.arm()
        self.info("Arm complete")

        while self.armed:
            time.sleep(1)

        self.info("Disarmed") 

        for t in self.threads:
            t.join()

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
            att.q,#[0,0,0,0], # att.q,IGNORE ATTITUTDE
            att.body_roll_rate,
            att.body_pitch_rate,
            att.body_yaw_rate,
            att.thrust
        )

    def time_ms(self):
        return time.time() * 1000.0

    def controller(self):
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


class GuidedPX4Quadrotor(PX4Quadrotor):

    def __init__(self, *args):
        super(GuidedPX4Quadrotor, self).__init__(*args)

    def waypoint_reached_callback(self, number, pt):
        self.info("Gate {} passed at {} ".format(number, pt))
        if number == len(self.track.gates)-1:
            self.info("Race stopped")
            self.record = False 
            self.race_end = True
            self.lap_time = self.current_time - self.time_start


    def land(self):
        #Land
        # TODO Why does this trigger land? Bit mask not set
        # maybe due to altitude
        land_pt = SimplePointGate(self.flight_trajectory[-1].x, self.flight_trajectory[-1].y, -1 * self.flight_trajectory[-1].z)
        self.set_location(land_pt)

    def mission_loop(self, gate_callback=None):
        # Way points are local and relative
        gates = self.track.gates
        for i in range(len(gates)):
            if not self.running:
                break

            gate = gates[i]
            print "Next gate is at ", gate
            self.set_location(gate)
            if gate_callback:
                gate_callback(i, gate)

        self.info("All waypoints reached, mission thread complete")
        #self.land()

        
    def set_location(self, gate):
        """
        Monitor the progress and set the next point 
        once reached
        """
        # While we have not reached our destination keep setting our next set point
        # at the predefined rate
        while self.running:
            start = time.time() # TODO move to _d?
            # TODO add more constraints
            if gate.detected(self.flight_trajectory[-1].x, self.flight_trajectory[-1].y, self.flight_trajectory[-1].z):
                break
            self.set_position_target_local_ned(gate.x, gate.y, gate.z)
            self._sleep(start, self.SET_PT_RATE)


    def arm_and_begin(self):

        """
        t = threading.Thread(target=self.keep_in_offboard_mode)
        t.start()
        self.threads.append(t)
        """

        t = threading.Thread(target=self.mission_loop, args=(self.waypoint_reached_callback,))
        t.start()
        self.threads.append(t)
        """
        while self.mode.name != "OFFBOARD":
            self.info("Waiting for mode change, current mode={}".format(self.mode.name))
            time.sleep(1)
        """

        while self.mode.name != "OFFBOARD":
            self._master.set_mode_px4('OFFBOARD', None, None)
            self.info("Waiting for mode change, current mode={}".format(self.mode.name))
            time.sleep(1)

        self.info("Arming...")
        self.arm()
        self.info("Arming complete")

    def fly(self, track, input=None):
        self.track = track

        self.set_max_horizontal_velocity(1.0)
        self.running = True
        self.arm_and_begin()

        while self.armed:
            time.sleep(0.5)


        self.info("Disarmed, wait...")
        
        for t in self.threads:
            t.join()

        self.info("All threads joined")
        #self.flight_data = FlightData(self.track.gates, self.flight_trajectory, self.flight_attitudes)



class Evolver(object):
    """ Radius in meters that is accepted to hit the waypoint """
    WAYPOINT_R = 1.0


    def __init__(self, gazebo_host="127.0.0.1", gazebo_port=11345, px4_host="127.0.0.1", px4_port=14540):
        self.px4_connect_string = "{}:{}".format(px4_host, px4_port)
        self.gazebo_host = gazebo_host
        self.gazebo_port = gazebo_port

        self.gz = GazeboAPI(gazebo_host, gazebo_port)

        self.flight_data = []
        self.flight_data_attitudes = []
        self.flight_data_paths = []

        self.track = straight_line_track(num_gates = 2, altitude = -2)

        self.gate_times = []
        logger.info("Race track: {}".format(self.track))
        
        self.racing = False



    def init_search(self):
	# The weights tuple depends on what is returned in the evaluation function
        # We will want to minimize the time
	creator.create("FitnessMin", base.Fitness, weights=(-1.0,))
        creator.create("Individual", list, fitness=creator.FitnessMin)

        self.toolbox = base.Toolbox()
        # Attribute generator
        self.toolbox.register("attr_item", self.get_random_char)

        self.toolbox.register("word", self.gen_filter, self.MIN_LENGTH, self.MAX_LENGTH)
        # Structure initializers
        self.toolbox.register("individual", tools.initIterate, creator.Individual, self.toolbox.word)
        self.toolbox.register("population", tools.initRepeat, list, self.toolbox.individual)
        # Operator registering
        self.toolbox.register("evaluate",self.eval)
        self.toolbox.register("mate", self.mate)
        self.toolbox.register("mutate",self.mut)
        self.toolbox.register("addFilter", self.mutAddFilter)
        self.toolbox.register("delFilter", self.mutDelFilter)
        self.toolbox.register("select",tools.selBest )

    def _model_reset_callback(self):
        logger.info("Model reset")


    def _has_fastest_time(self, vehicle):
        """ At anytime we just want to take a snapshot and see if they are going faster. Two cases, drone made it to the gate but not faster, drone still hasnt made it to the gate (we missed a sample) """
        is_fastest = True

        """
        if (len(vehicle.gate_times) == 0 and
            vehicle.time_last_attitude_sample > vehicle.track.gates[0].time_best):
            is_fastest = False
        else:
        """
        #print "V gate times ", vehicle.gate_times, " Best ", vehicle.track.gates

        number_gate_times = len(vehicle.gate_times)
        if number_gate_times < vehicle.track.gate_count: # Havnt hit all gates
            last_gate_best_time = vehicle.track.gates[number_gate_times].time_best
            #print "\tLast sample time ", vehicle.time_last_attitude_sample 
            #time_last_attitude_sample
            if vehicle.time_since_takeoff > last_gate_best_time:
                is_fastest = False

        else:
            for i in range(vehicle.track.gate_count):
                if vehicle.gate_times[i] > vehicle.track.gates[i].time_best:
                    is_fastest = False
                    break;
        return is_fastest

    def race_timeout(self, vehicle):
        logger.info("Waiting for race timeout")
        end = False
        while True:#self.racing:
            if vehicle.track:
                    #(vehicle.track.gate_count > vehicle.gate_next and # There are still gates left
                    #vehicle.time_since_armed  > vehicle.track.gates[vehicle.gate_next].time_best) 
                if not self._has_fastest_time(vehicle):
                    logger.info("Didnt beat best times of {} with times {} ".format(vehicle.track.gates, vehicle.gate_times))
                    end = True

                elif (vehicle.track.gate_count == vehicle.gate_next): #completed
                    logger.info("Completed all gates {} {}".format(vehicle.track.gate_count,vehicle.gate_next))
                    end = True

                if end:
                    vehicle.running = False #stop all threads
                    vehicle.armed = False
                    #vehicle.close()
                    break
            time.sleep(1)
            #logger.info("Everything ok t={} {} {}".format(vehicle.time_since_takeoff, vehicle.track.gates, vehicle.gate_times))

        self.racing = False
        logger.info("outside loop")

    def run(self):
        t, baseline_attitudes = self.race("baseline", GuidedPX4Quadrotor)
        logger.info("Lap Time={}".format(t))

        logged_att_sp = self.parse_log()
        #self.flight_data_attitudes.append(logged_att_sp)
        self.flight_data.append(FlightData("logged-baseline", None, None, logged_att_sp))

        logger.info("EVOLVING")
        t2, evolved_attitues = self.race("e1", EvolvedQuadrotor, input=logged_att_sp)

        logged_att_sp = self.parse_log()
        self.flight_data.append(FlightData("logged-e1", None, None, logged_att_sp))

        
        #print "Len: ", len(self.flight_data_attitudes)
        #data = FlightAnalysis(self.gate_times, self.track.gates, self.flight_data_paths, self.flight_data_attitudes)
        data = FlightAnalysis()
        
        data.plot_input(self.flight_data)
        
        data.show()



    def status_printer(self, txt):
        #Limited to messages of 50 characters
        loggerPX4.info(txt)

    def race(self, name, vehicle_class, input=None):
        v = connect(self.px4_connect_string, wait_ready=True, vehicle_class=vehicle_class, status_printer=None)

        self.racing = True
        t = threading.Thread(target=self.race_timeout, args=(v,))#, args=(self.waypoint_reached_callback,))
        t.start()

        v.fly(self.track, input)
        # Wait in main thread
        while self.racing:
            time.sleep(1)
        logger.info("Joining...")
        t.join()

        v.close()


        #inputs = v.control_inputs()
        logger.info("Total attitude points collect={}".format(len(v.flight_attitudes)))
        #logger.info("Roll={} Pitch={} Yaw={} Thrust={}".format(len(inputs.roll), len(inputs.pitch), len(inputs.yaw), len(inputs.thrust)))

        #data = FlightData(name, v.gate_times,v.flight_trajectory, v.flight_attitudes)

        #self.flight_data_attitudes.append(v.flight_attitudes)
        #self.flight_data_paths.append(v.flight_trajectory)
        #self.flight_data.append(data)

        self.gate_times.append(v.gate_times)
        #v.diagnostics()


        # Update the best times
        self.track = v.track


        logger.info("Race stats {}".format(self.track))
        GazeboAPI(self.gazebo_host, self.gazebo_port).reset_model(self._model_reset_callback)


        return (v.time_lap, v.flight_attitudes )


    def get_last_log(self):
        path = "/home/wil/workspace/buflightdev/PX4/build_posix_sitl_lpe/tmp/rootfs/fs/microsd/log/2017-08-29/*.ulg"
        list_of_files = glob.glob(path)
        return max(list_of_files, key=os.path.getctime)


    def parse_log(self):
        """
        Return a list of Attitude_SP
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
                attitude_sp.append(Attitude_SP(
                                          t, 
                                          d.data["roll_body"][i],
                                          d.data["pitch_body"][i],
                                          d.data["yaw_body"][i], 
                                          d.data["thrust"][i],
                                        [d.data["q_d[0]"][i], d.data["q_d[1]"][i], d.data["q_d[2]"][i], d.data["q_d[3]"][i]]
                                        ))
        return attitude_sp

class Attitude_SP:
    def __init__(self, time_boot_ms, body_roll_rate, body_pitch_rate, body_yaw_rate, thrust, q):
        self.time_boot_ms = time_boot_ms
        self.body_roll_rate = body_roll_rate
        self.body_pitch_rate = body_pitch_rate
        self.body_yaw_rate = body_yaw_rate
        self.thrust = thrust
        self.q = q

def fitness():
    """
    1. time
    2. minimize points and smoothness
    """
    pass

if __name__ == "__main__":
    #connection_string = '127.0.0.1:14540'
    # Connect to the Vehicle
    #print "Connecting"
    #v = connect(connection_string, wait_ready=True, vehicle_class=PX4Quadrotor)
    #pilot = FastVehicle()
    #time.sleep(10)
    init_logging()
    e = Evolver()
    e.run()
    #e.parse_log()



