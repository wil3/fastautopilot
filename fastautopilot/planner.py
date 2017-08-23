
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

        self.time_start = None

        self.time_lap = None

        self.threads = []

        """ Current mode to detect when we change """
        self.curr_mode = None

        self.running = False

        self.race_end = False

        self.record = False


        """ The track the MAV will fly """
        self.track = None

        """ Save to check for transitions, i.e., takeoff or land """
        self.curr_land_state = None

        """ Instance of FlightData """
        self.fight_data = None

        """
        Each message is sent at a certain rate as defined in configure_stream
        src/modules/mavlink/mavlink_main.cpp
        """

        @self.on_message('SYSTEM_TIME')
        def system_time_listener(self, name, time):
            self.current_time = time.time_unix_usec

        @self.on_message('ATTITUDE_TARGET')
        def attitude_target_listener(self, name, target):
            if self.armed and self.record:
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
                print "RACE BEGIN!"
                self.record = True
                self.time_start = self.current_time

            self.curr_land_state = state.landed_state

        @self.on_message('LOCAL_POSITION_NED')
        def local_position_ned_listener(self, name, data):
            #print data
            #pass
            self.flight_trajectory.append(data)


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
        while not self.armed:      
                time.sleep(0.5)

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

class GuidedPX4Quadrotor(PX4Quadrotor):

    def __init__(self, *args):
        super(GuidedPX4Quadrotor, self).__init__(*args)

    def waypoint_reached_callback(self, number, pt):
        print "Waypoint reached ", pt
        if number == 0: # First waypoint reached, ie has taken off
            #self.record = True #Start recording
            pass
        #elif number == (len(self.waypoints()) -2):
        elif number == len(self.track.gates)-1:
            print "Race stopped"
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

        print "All waypoints reached"
        self.land()

        
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

        t = threading.Thread(target=self.keep_in_offboard_mode)
        t.start()
        self.threads.append(t)

        t = threading.Thread(target=self.mission_loop, args=(self.waypoint_reached_callback,))
        t.start()
        self.threads.append(t)

        while self.mode.name != "OFFBOARD":
            print "Waiting ", self.mode.name
            time.sleep(1)

        print "Arming..."
        #self.arm = True
        self.arm()
        print "Arming complete"

    def fly(self, track):
        self.track = track
        self.set_max_horizontal_velocity(1.0)
        self.running = True
        self.arm_and_begin()

        while self.armed:
            time.sleep(1)
        
        for t in self.threads:
            t.join()

        print "All threads joined"
        self.flight_data = FlightData(self.track.gates, self.flight_trajectory, self.flight_attitudes)



class Evolver(object):
    """ Radius in meters that is accepted to hit the waypoint """
    WAYPOINT_R = 1.0

    def __init__(self, gazebo_host="127.0.0.1", gazebo_port=11345, px4_host="127.0.0.1", px4_port=14540):
        self.px4_connect_string = "{}:{}".format(px4_host, px4_port)

        self.gz = GazeboAPI(gazebo_host, gazebo_port)
        
    def generate_baseline(self):
        v = connect(self.px4_connect_string, wait_ready=True, vehicle_class=GuidedPX4Quadrotor)

        track = straight_line_track(num_gates = 3)
        print track
        v.fly(track)
        inputs = v.control_inputs()
        logger.info("Total attitude points collect={}".format(len(v.flight_attitudes)))
        logger.info("Roll={} Pitch={} Yaw={} Thrust={}".format(len(inputs.roll), len(inputs.pitch), len(inputs.yaw), len(inputs.thrust)))

        v.diagnostics()
        return (v.time_lap, inputs)


    def _model_reset_callback(self):
        logger.info("Model reset")

    def evolve(self):
        t, baseline_inputs = self.generate_baseline()
        logger.info("Lap Time={}".format(t))
        self.gz.reset_model(self._model_reset_callback)


if __name__ == "__main__":
    #connection_string = '127.0.0.1:14540'
    # Connect to the Vehicle
    #print "Connecting"
    #v = connect(connection_string, wait_ready=True, vehicle_class=PX4Quadrotor)
    #pilot = FastVehicle()
    #time.sleep(10)
    init_logging()
    e = Evolver()
    e.evolve()



