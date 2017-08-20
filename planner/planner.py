from sim.gazebo.gazeboapi import GazeboWorldStats

from dronekit import connect, Command, LocationGlobal, Vehicle, VehicleMode
from pymavlink import mavutil
import time, sys, argparse, math

from tracker import *
from mission import set_mission
import threading

class PX4Quadrotor(Vehicle):
    """
    Copter is contolled using SET_POSITION_TARGET_LOCAL_NED and SET_ATTITUDE_TARGET 

    https://github.com/PX4/Firmware/issues/1288

    For offboard 

    http://discuss.px4.io/t/offboard-automatic-takeoff-landing-using-mav-cmd-land-takeoff-local/1333/3

    MUST PUBLISH FASTER THAN  2HZ TO STAY IN OFFBOARD
    https://dev.px4.io/en/ros/mavros_offboard.html
    """

    MAV_MODE_AUTO   = 4
    MAV_CMD_NAV_LAND_LOCAL = 23
    MAV_CMD_NAV_GUIDED_ENABLE = 92
    MAV_LANDED_STATE_ON_GROUND = 1
    MAV_MODE_GUIDED_ARMED = 216
    START_TIME = 0
    END_TIME = 0

    def __init__(self, *args):
        super(PX4Quadrotor, self).__init__(*args)


        self.home_position_set = False

        # Track the trajectory
        self.list_loc = []
        # Track the inputs
        self.list_att = []
        # The waypoint locations
        self.wp_loc = []

        self.current_time = None

        self.local_position_ned = None

        self.threads = []

        self.curr_mode = None

        self.target_loc = []

        self.running = False

        #self._master.mav.set_send_callback(self.send_callback)

        """
        Each message is sent at a certain rate as defined in configure_stream
        src/modules/mavlink/mavlink_main.cpp
        """

        @self.on_message('SYSTEM_TIME')
        def system_time_listener(self, name, time):
            self.current_time = time.time_unix_usec
            #print "T=", time.time_boot_ms   


        #Create a message listener for home position fix
        @self.on_message('HOME_POSITION')
        def listener(self, name, home_position):
            self.home_position_set = True

        @self.on_message('ATT_POS_MOCAP')
        def l(self,name, data):
            #print data
            pass
        #@self.on_message('HEARTBEAT')
        #def heartbeat(self, name, data):
            #print data
            pass
        @self.on_message('ATT_POS_MOCAP')
        def mocap(self, name, data):
            print data

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

            if self.armed:
                #if time_boot < 0:
                #    time_boot = target.time_boot_ms
                self.list_att.append(target)

        @self.on_message('EXTENDED_SYS_STATE')
        def landed_state_listener(self, name, state):
            if state.landed_state == self.MAV_LANDED_STATE_ON_GROUND: 
                #print "Drone on the ground"
                pass

        @self.on_message('GLOBAL_POSITION_INT')
        def gps_listener(self, name, data):
            #print "GPS ", data
            pass
        @self.on_message('LOCAL_POSITION_NED')
        def local_position_ned_listener(self, name, data):
            #print data
            #pass
            self.local_position_ned = data
            self.list_loc.append(data)

        @self.on_message('COMMAND_ACK')
        def ack(self, name, data):
            #print data
            pass

        @self.on_message('GPS_RAW_INT')
        def gps_listener(self, name, data):
            #print "GPS RAW", data
            pass
        @self.on_attribute('attitude')
        def attitude_listener(self, name, attitude):
            #print "yaw={} pitch={} roll={}".format(attitude.yaw, attitude.pitch, attitude.roll)
            #print "gps={}".format(vehicle.gps_0)
            pass


        @self.on_attribute('location')
        def location_listener(self, name, location):
            curr_lat = location.global_relative_frame.lat
            curr_lon = location.global_relative_frame.lon 
            curr_alt = location.global_relative_frame.alt
            if curr_lat == None or curr_lon == None or curr_alt == None:
               return 

            #only update if sometihng changed
            """
            if (len(lat) > 0 and lat[-1] == curr_lat and 
               len(lon) > 0 and lon[-1] == curr_lon and 
               len(alt) > 0 and alt[-1] == curr_alt):
                return 
            """
            if self.armed:
                #print "Lat={} Lon={} Alt={}".format(curr_lat, curr_lon, curr_alt)
                #self.list_loc.append(location.global_relative_frame)
                pass

        #time_boot = -1 


    def PX4setMode(self, mode, custom_mode):
        # The mode is specific to what the controller supports
        self._master.mav.command_long_send(self._master.target_system, self._master.target_component,
                                                   mavutil.mavlink.MAV_CMD_DO_SET_MODE, 
                                                   mode,
                                                   custom_mode,
                                                   0, 0, 0, 0, 0, 0)
    def mav_cmd_nav_land_local(self, x, y, z):
        self._master.mav.command_long_send(self._master.target_system, self._master.target_component,
                                           mavutil.mavlink.MAV_CMD_NAV_LAND_LOCAL, 0,
                                           0, 1, 0,
                                           x, y, z,0)

    def mav_cmd_nav_land(self, x, y, z):
        self._master.mav.command_long_send(self._master.target_system, self._master.target_component,
                                           mavutil.mavlink.MAV_CMD_NAV_LAND, 0,
                                           0, 0, 0,
                                           x, y, z,0)
    def mav_cmd_nav_takeoff_local(self, x, y, z):
        self._master.mav.command_long_send(self._master.target_system, self._master.target_component,
                                           mavutil.mavlink.MAV_CMD_NAV_TAKEOFF_LOCAL, 0,
                                           0, 10, 0,
                                           x, y, z,0)

    def mav_cmd_nav_guided_enable(self, enable=True):
        
        value = 0
        if enable:
            value = 1
        print self._master.mav.command_long_send(self._master.target_system, self._master.target_component,
                                           self.MAV_CMD_NAV_GUIDED_ENABLE, value,
                                           0, 0, 0, 0, 0, 0, 0)

    def keep_in_offboard_mode(self, rate):
        while self.running:
            self._master.set_mode_px4('OFFBOARD', None, None)
            time.sleep(1/float(rate))
            if not(self.mode == self.curr_mode):
                print "Changed modes ",self.mode
                self.curr_mode = self.mode

    def set_position_target_local_ned(self):
        self._master.mav.set_position_target_local_ned_send(
            0, #time boot not used
            self._master.target_system,
            self._master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111111000,
            0, 0, -10, # x y z in m
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

    def goto_position_target_local_ned(self, north, east, down):
        """	
        Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified 
        location in the North, East, Down frame.
        It is important to remember that in this frame, positive altitudes are entered as negative 
        "Down" values. So if down is "10", this will be 10 metres below the home altitude.
        Starting from AC3.3 the method respects the frame setting. Prior to that the frame was
        ignored. For more information see: 
        http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_local_ned
        See the above link for information on the type_mask (0=enable, 1=ignore). 
        At time of writing, acceleration and yaw bits are ignored.
        """
        msg = self.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
            0b0000111111111000, # type_mask (only positions enabled)
            north, east, down, # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
            0, 0, 0, # x, y, z velocity in m/s  (not used)
            0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
        # send command to vehicle
        self.send_mavlink(msg)

    def send_callback(self, msg):
        print "Message ", msg

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
                print " Waiting for arming..."
                time.sleep(1)

    def set_point(self):
        while self.running:
            self.takeoff()
            time.sleep(0.1)

    def arm_and_takeoff(self):
        self.target_loc = [0, 0, -2]
        #while not self.home_position_set:
        #    print "Waiting for home position..."
        #    time.sleep(1)

        #cmds = self.commands
        #cmds.clear()
        #self.mode = VehicleMode("GUIDED")
        #self.PX4setMode(mavutil.mavlink.MAV_MODE_GUIDED_ARMED, 0)
        #self.PX4setMode(0, self.MAV_MODE_AUTO)
        print "Priming..."
        i = 0
        for i in range(100):
            self.takeoff()
            time.sleep(0.1)

        """
        t = threading.Thread(target=self.set_point)
        t.start()
        self.threads.append(t)
        time.sleep(5)
        """
        
        #First send data points
        print "Done priming setpoint"

        #for i in range(4):

        """
        while self.mode.name != "OFFBOARD":
            print "Waiting ", self.mode.name
            time.sleep(1)
            """
        print "Mode set to ", self.mode.name

        #time.sleep(5)

        #self.groundspeed = 5
        t = threading.Thread(target=self.keep_in_offboard_mode, args=(4,))
        t.start()
        self.threads.append(t)

        t = threading.Thread(target=self.set_point)
        t.start()
        self.threads.append(t)

        while self.mode.name != "OFFBOARD":
            print "Waiting ", self.mode.name
            time.sleep(1)

        #print "Mode set to ", self.mode.name 

        print "Arming..."
        self.arm()
        print "Arming complete"

        print "Taking off ", self.mode.name
        #time.sleep(10)
        #self.mav_cmd_nav_guided_enable()
        #time.sleep(1)
        #time.sleep(1)
        #targetAltitude = 5 
        time.sleep(10)
        #self.groundspeed = 5
        #self.airspeed = 5
        #self.set_position_target_local_ned()
        #for i in range(10):
        #    self.takeoff()
        #    time.sleep(0.25)
        #time.sleep(5)
        #self.simple_takeoff(targetAltitude)
        #self.mav_cmd_nav_takeoff_local(0, 0, -targetAltitude)

    def shutdown(self):
        self.running = False
        for t in self.threads:
            t.join()

    def land(self):
        print("Setting LAND mode...")
        #self.mode = VehicleMode("LAND")
        self._master.set_mode_px4('LAND', None, None)
        #self.mav_cmd_nav_land(0, 0, 0)
        #self.mav_cmd_nav_land_local(1, 1, 0)
        #time.sleep(3)
        #self.armed = False
        time.sleep(1)
        self.close()
        time.sleep(1)
        print "Vehicle closed"

    def fly(self):
        self.running = True
        self.arm_and_takeoff()
        time.sleep(3)
        print "Landing"
        #self.land()
        self.shutdown()

    def square(self):


	print("SQUARE path using SET_POSITION_TARGET_LOCAL_NED and position parameters")
	DURATION = 20 #Set duration for each segment.

	print("North 50m, East 0m, 10m altitude for %s seconds" % DURATION)
	self.goto_position_target_local_ned(50,0,-10)
	print("Point ROI at current location (home position)") 
# NOTE that this has to be called after the goto command as first "move" command of a particular type
# "resets" ROI/YAW commands
	#self.set_roi(vehicle.location.global_relative_frame)
	time.sleep(DURATION)

	print("North 50m, East 50m, 10m altitude")
	self.goto_position_target_local_ned(50,50,-10)
	time.sleep(DURATION)

	print("Point ROI at current location")
	#self.set_roi(vehicle.location.global_relative_frame)

	print("North 0m, East 50m, 10m altitude")
	self.goto_position_target_local_ned(0,50,-10)
	time.sleep(DURATION)

	print("North 0m, East 0m, 10m altitude")
	self.goto_position_target_local_ned(0,0,-10)
	time.sleep(DURATION)




    def fly2(self):

        self.arm_and_takeoff()
        self.square()
        # wait for a home position lock
        #while not self.home_position_set:
        #    print "Waiting for home position..."
        #    time.sleep(1)

        # Change to AUTO mode
        #self.PX4setMode(self.MAV_MODE_AUTO)
        #time.sleep(3)
        #self.arm()
        #targetAltitude = 10 

        #self.simple_takeoff(targetAltitude)
        #self.mav_cmd_nav_takeoff_local(0, 0, targetAltitude)
        #print "Takeing off"
        #time.sleep(5)

        self.mav_cmd_nav_guided_enable()
        time.sleep(1)
        self.PX4setMode(self.MAV_MODE_GUIDED_ARMED)
        time.sleep(1)
        print "Mode ", self.mode


        self.goto_position_target_local_ned(0, 1, 0)
        time.sleep(1)
        #self.goto_position_target_local_ned(0, -1, 0)
        #time.sleep(1)
        self.mav_cmd_nav_land_local(1, 1, 0)

        #set_mission(self)
# Load commands
        time.sleep(2)

# Arm vehicle
#world_stat = GazeboWorldStats("localhost", 11345)
#world_stat.get_sim_time(start_sim_time_callback)

# monitor mission execution
        """
        nextwaypoint = self.commands.next
        while nextwaypoint < len(self.commands):
            if self.commands.next > nextwaypoint:
                display_seq = self.commands.next+1
                print "Moving to waypoint %s" % display_seq
                nextwaypoint = self.commands.next
            time.sleep(1)
# wait for the vehicle to land
        while self.commands.next > 0:
            time.sleep(1)
        """
# Disarm vehicle
        self.armed = False

#world_stat.get_sim_time(end_sim_time_callback)

        time.sleep(1)
# Close vehicle object before exiting script
        self.close()
        time.sleep(1)

        print "Vehicle closed"
#p.join()



    def print_stats(self):
        data = FlightData()
        data.trajectory(self.list_loc,self.wp_loc)
        data.inputs(self.list_att)
        data.show()

if __name__ == "__main__":
    connection_string = '127.0.0.1:14540'
    # Connect to the Vehicle
    print "Connecting"
    v = connect(connection_string, wait_ready=True, vehicle_class=PX4Quadrotor)
    #pilot = FastVehicle()
    v.fly()
    #v.print_stats()
    #time.sleep(10)

