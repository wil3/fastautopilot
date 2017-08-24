
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

def init_logging():
    log_config = os.path.join(".", "../conf/logging.yaml")
    if os.path.exists(log_config):
        with open(log_config, 'rt') as f:
            log_config = yaml.load(f.read())
        logging.config.dictConfig(log_config)
    else:
        raise Exception("Cannot find log configuration file")

logger = logging.getLogger('fastautopilot')


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

        """
        Each message is sent at a certain rate as defined in configure_stream
        src/modules/mavlink/mavlink_main.cpp
        """

        @self.on_message('SYSTEM_TIME')
        def system_time_listener(self, name, t):
            self.current_time = t.time_unix_usec
            self.time_boot_ms = t.time_boot_ms

            if self.armed and not self.time_armed:
                self.time_armed = t.time_boot_ms

            if self.armed and self.time_armed:
                self.time_since_armed = self.time_boot_ms - self.time_armed 

        @self.on_message('ATTITUDE_TARGET')
        def attitude_target_listener(self, name, target):
            if self.armed and self.time_since_armed:# and self.record:
                target.time_boot_ms = self.time_since_armed
                self.flight_attitudes.append(target)
                

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
            #print data
            #pass
            self.flight_trajectory.append(data)

            # TODO move to gate?
            if self.track:
                
                if (self.track.gate_count > self.gate_next and
               self.track.gates[self.gate_next].detected(data.x, data.y, data.z)):

                    gate_time = self.time_since_armed

                    self.info("Gate {} reached at {}".format(self.gate_next, self.time_since_armed))
                   
                    self.gate_times.append(gate_time)

                    # Update if best time
                    if not self.track.gates[self.gate_next].time_best or self.track.gates[self.gate_next].time_best > gate_time:
                        self.track.gates[self.gate_next].time_best = gate_time
                    self.gate_next += 1 
                

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
        while not self.armed and not self.time_since_armed:      
                time.sleep(0.1)


    def info(self, message):
        logger.info("[{}]\t{}".format(self.time_since_armed, message))

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

        t = threading.Thread(target=self.autopilot)#, args=(self.waypoint_reached_callback,))
        t.start()
        self.threads.append(t)


        while self.mode.name != "OFFBOARD":
            self._master.set_mode_px4('OFFBOARD', None, None)
            self.info("Waiting for mode change, current mode={}".format(self.mode.name))
            time.sleep(1)

        self.info("Arming...")
        self.arm()
        self.info("Arm complete")

        while self.armed:
            time.sleep(1)

        self.info("Disarmed") 

        for t in self.threads:
            t.join()

    def set_attitude_target(self, att, mask=0b00000000): 
        self._master.mav.set_attitude_target_send(
            0, 
            0, 
            0,
            mask, 
            att.q,
            att.body_roll_rate,
            att.body_pitch_rate,
            att.body_yaw_rate,
            att.thrust
        )

    def autopilot(self):
        """
        roll = deque(self.input.roll)
        pitch = deque(self.input.pitch)
        yaw = deque(self.input.yaw)
        thrust = deque(self.input.thrust)
        """
        attitudes = deque(self.input)
        
        curr_attitude = attitudes.popleft()
        next_attitude = attitudes.popleft()
        self.info("First attitude {}".format(curr_attitude))

        while self.running:

            if self.armed:
                if  attitudes and next_attitude.time_boot_ms <= self.time_since_armed:
                    #self.info("Setting attitude at t={} {}".format(normal, next_attitude))
                    curr_attitude = next_attitude
                    next_attitude = attitudes.popleft()

                self.set_attitude_target(curr_attitude)

            else:
                # Keep in OFFBOARD mode
                self.set_attitude_target(next_attitude, 0b11111111)

            


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

        self.flight_data_attitudes = []
        self.flight_data_paths = []

        self.track = straight_line_track(num_gates = 1, altitude = -2)
        logger.info("Race track: {}".format(self.track))
        
        self.racing = False

    def race(self, vehicle_class, input=None):
        v = connect(self.px4_connect_string, wait_ready=True, vehicle_class=vehicle_class)

        self.racing = True
        t = threading.Thread(target=self.race_timeout, args=(v,))#, args=(self.waypoint_reached_callback,))
        t.start()

        v.fly(self.track, input)
        # Wait in main thread
        while self.racing:
            time.sleep(1)
        t.join()

        v.close()


        #inputs = v.control_inputs()
        logger.info("Total attitude points collect={}".format(len(v.flight_attitudes)))
        #logger.info("Roll={} Pitch={} Yaw={} Thrust={}".format(len(inputs.roll), len(inputs.pitch), len(inputs.yaw), len(inputs.thrust)))

        self.flight_data_attitudes.append(v.flight_attitudes)
        self.flight_data_paths.append(v.flight_trajectory)

        #v.diagnostics()


        # Update the best times
        self.track = v.track


        logger.info("Race stats {}".format(self.track))
        GazeboAPI(self.gazebo_host, self.gazebo_port).reset_model(self._model_reset_callback)

        return (v.time_lap, v.flight_attitudes )

    def _model_reset_callback(self):
        logger.info("Model reset")


    def _has_fastest_time(self, vehicle):
        """ At anytime we just want to take a snapshot and see if they are going faster """
        is_fastest = True

        if (len(vehicle.gate_times) == 0 and
            vehicle.time_since_armed > vehicle.track.gates[0].time_best):
            is_fastest = False
        else:
            for i in range(len(vehicle.gate_times)):
                if vehicle.gate_times[i] > vehicle.track.gates[i].time_best:
                    is_fastest = False
                    break;
        return is_fastest

    def race_timeout(self, vehicle):
        logger.info("Waiting for race timeout")
        while True:#self.racing:
            if (
                vehicle.track and
                (
                    #(vehicle.track.gate_count > vehicle.gate_next and # There are still gates left
                    #vehicle.time_since_armed  > vehicle.track.gates[vehicle.gate_next].time_best) 
                    not self._has_fastest_time(vehicle)
                    or
                    (vehicle.track.gate_count == vehicle.gate_next)
                )
            ):
                logger.info("Didnt beat best time, or race over closing")
                vehicle.running = False #stop all threads
                vehicle.armed = False
                #vehicle.close()
                break
            time.sleep(0.1)

        self.racing = False
        logger.info("outside loop")

    def run(self):
        t, baseline_attitudes = self.race(GuidedPX4Quadrotor)
        logger.info("Lap Time={}".format(t))

        logger.info("EVOLVING")
        t2, evolved_attitues = self.race(EvolvedQuadrotor, input=baseline_attitudes)
        
        data = FlightData(self.track.gates, self.flight_data_paths, self.flight_data_attitudes)
        data.show()


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



