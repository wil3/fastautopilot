
import logging.config
from dronekit import connect, Command, LocationGlobal, Vehicle, VehicleMode
from pymavlink import mavutil
import time, sys, argparse, math, os, yaml
import numpy as np
from tracker import *
import threading
from collections import deque

from sim.gazebo.gazeboapi import GazeboAPI 

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

class GuidedPX4Quadrotor(Vehicle):
    """
    Copter is contolled using SET_POSITION_TARGET_LOCAL_NED and SET_ATTITUDE_TARGET 

    https://github.com/PX4/Firmware/issues/1288

    For offboard,
    http://discuss.px4.io/t/offboard-automatic-takeoff-landing-using-mav-cmd-land-takeoff-local/1333/3

    MUST PUBLISH FASTER THAN  2HZ TO STAY IN OFFBOARD
    https://dev.px4.io/en/ros/mavros_offboard.html
    """

    START_TIME = 0
    END_TIME = 0

    PT_R = 1.0
    SAMPLE_RATE = 10 
    SET_PT_RATE = 20

    INIT_ALTITUDE = -2

    def __init__(self, *args):
        super(GuidedPX4Quadrotor, self).__init__(*args)

        # Track the trajectory
        self.flight_trajectory = []
        # Track the inputs
        self.flight_attitudes = []
        # The waypoint locations
        self.wp_loc = []

        self.current_time = None

        self.local_position_ned = None

        self.threads = []

        self.curr_mode = None

        self.target_pos = []

        self.running = False

        self.race_end = False

        self.record= False

        self.curr_land_state = None

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
            #global time_boot 


            """
            if (len(roll) > 0 and roll[-1] == target.body_roll_rate and
                len(pitch) > 0 and pitch[-1] == target.body_pitch_rate and
                len(yaw) > 0 and yaw[-1] == target.body_yaw_rate and
                len(thrust) > 0 and thrust[-1] == target.thrust):
                return
            """

            if self.armed and self.record:
                #if time_boot < 0:
                #    time_boot = target.time_boot_ms
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

            self.curr_land_state = state.landed_state

        @self.on_message('LOCAL_POSITION_NED')
        def local_position_ned_listener(self, name, data):
            #print data
            #pass
            self.local_position_ned = data
            self.flight_trajectory.append(data)

    def keep_in_offboard_mode(self, rate):
        while self.running:
            self._master.set_mode_px4('OFFBOARD', None, None)
            time.sleep(1/float(rate))
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
            0b0000111111111000,
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
        START_TIME = sim_time.sec
        
    def end_sim_time_callback(self, sim_time):
        print "End Sim Time", sim_time.sec
        END_TIME = sim_time.sec
        print "Lapse time ", END_TIME - START_TIME

    def arm(self):
        self.armed = True 
        while not self.armed:      
                time.sleep(0.5)


    def _d(self, pt1, pt2):
        """
        Return distance between two points
        """
        a = np.array(pt1)
        b = np.array(pt2)
        d = np.linalg.norm(a-b)
        return d



    def _sleep(self, start, rate):
        """
        Sleep for specified time based on the starting time
        and the rate in which command should execute
        """
        lapse = time.time() - start
        T = 1/float(self.SET_PT_RATE)
        if lapse < T:
            time.sleep(T - lapse)



    def waypoint_reached_callback(self, number, pt):
        print "Waypoint reached ", pt
        if number == 0: # First waypoint reached, ie has taken off
            #self.record = True #Start recording
            pass
        elif number == (len(self.waypoints()) -2):
            print "Race stopped"
            self.record = False 
            self.race_end = True

    def mission_loop(self, callback=None):
        # Way points are local and relative
        wp = self.waypoints()
        count = 0
        while self.running and wp:
            set_pt = wp.popleft()
            print "Next set point ", set_pt
            self.set_point(set_pt)
            if callback:
                callback(count, set_pt)
            count += 1 

        print "All waypoints reached"

        #Land
        # TODO Why does this trigger land? Bit mask not set
        # maybe due to altitude
        land_pt = [self.flight_trajectory[-1].x, self.flight_trajectory[-1].y, -1 * self.flight_trajectory[-1].z]
        self.set_point(land_pt)

        
    def set_point(self, set_pt):
        """
        Monitor the progress and set the next point 
        once reached
        """
        # While we have not reached our destination keep setting our next set point
        # at the predefined rate
        while self.running:
            start = time.time() # TODO move to _d?
            if self._d(set_pt,[self.flight_trajectory[-1].x, self.flight_trajectory[-1].y, self.flight_trajectory[-1].z]) <= self.PT_R:
                break
            self.set_position_target_local_ned(set_pt[0], set_pt[1], set_pt[2])
            self._sleep(start, self.SET_PT_RATE)


    def arm_and_begin(self):

        print "Mode set to ", self.mode.name
        t = threading.Thread(target=self.keep_in_offboard_mode, args=(4,))
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



    def waypoints(self):
        # Length of way points must be greater than 2!
        #return self.wp_pentagon()
        return self.wp_line()

    def wp_line(self):
        return deque([ 
                [0, 0, -2], # takeoff
                [10, 0, -2],
                [20, 0, -2],
        #        [30, 0, -2],
        #        [40, 0, -2],
        ])

    def wp_pentagon(self):
        return deque([ 
                [0, 0, self.INIT_ALTITUDE], # takeoff
                [10, 0, -2],
                [15, 5, -2],
                [10, 10,-2],
                [0, 10, -2],
                [-10, 10, -2],
        ])

    def fly(self):
        self.set_max_horizontal_velocity(1.0)
        self.running = True
        self.arm_and_begin()

        while self.armed:
            time.sleep(1)
        
        for t in self.threads:
            t.join()

        print "All threads joined"
        self.flight_data = FlightData(self.waypoints(), self.flight_trajectory, self.flight_attitudes)

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



class Evolver(object):
    def __init__(self, gazebo_host="127.0.0.1", gazebo_port=11345, px4_host="127.0.0.1", px4_port=14540):
        self.px4_connect_string = "{}:{}".format(px4_host, px4_port)

        self.gz = GazeboAPI(gazebo_host, gazebo_port)
        
    def generate_baseline(self):
        v = connect(self.px4_connect_string, wait_ready=True, vehicle_class=GuidedPX4Quadrotor)
        v.fly()
        inputs = v.control_inputs()
        logger.info("Total attitude points collect={}".format(len(v.flight_attitudes)))
        logger.info("Roll={} Pitch={} Yaw={} Thrust={}".format(len(inputs.roll), len(inputs.pitch), len(inputs.yaw), len(inputs.thrust)))

        #v.diagnostics()
        return inputs


    def _model_reset_callback(self):
        logger.info("Model reset")

    def evolve(self):
        baseline_inputs = self.generate_baseline()
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
