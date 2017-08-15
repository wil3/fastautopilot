from sim.gazebo.gazeboapi import GazeboWorldStats

from dronekit import connect, Command, LocationGlobal, Vehicle
from pymavlink import mavutil
import time, sys, argparse, math

from tracker import *
from mission import set_mission

class FastVehicle(Vehicle):

    MAV_MODE_AUTO   = 4
    MAV_LANDED_STATE_ON_GROUND = 1
    START_TIME = 0
    END_TIME = 0

    def __init__(self, *args):
        super(FastVehicle, self).__init__(*args)


        self.home_position_set = False

        # Track the trajectory
        self.list_loc = []
        # Track the inputs
        self.list_att = []
        # The waypoint locations
        self.wp_loc = []



################################################################################################
# Listeners
################################################################################################


#Create a message listener for home position fix
        @self.on_message('HOME_POSITION')
        def listener(self, name, home_position):
            self.home_position_set = True

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
                self.list_loc.append(location.global_relative_frame)

        #time_boot = -1 

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
                print "Drone on the ground"

    def PX4setMode(self, mavMode):
        self._master.mav.command_long_send(self._master.target_system, self._master.target_component,
                                                   mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                                                   mavMode,
                                                   0, 0, 0, 0, 0, 0)
    def start_sim_time_callback(self, sim_time):
        print "Start Sim Time", sim_time.sec
        START_TIME = sim_time.sec
        
    def end_sim_time_callback(self, sim_time):
        print "End Sim Time", sim_time.sec
        END_TIME = sim_time.sec
        print "Lapse time ", END_TIME - START_TIME

    def fly(self):
################################################################################################
# Start mission example
################################################################################################

# wait for a home position lock
        while not self.home_position_set:
            print "Waiting for home position..."
            time.sleep(1)



# Display basic vehicle state
        print " Type: %s" % self._vehicle_type
        print " Armed: %s" % self.armed
        print " System status: %s" % self.system_status.state
        print " GPS: %s" % self.gps_0
        print " Alt: %s" % self.location.global_relative_frame.alt

# Change to AUTO mode
        self.PX4setMode(self.MAV_MODE_AUTO)
        time.sleep(1)

        set_mission(self)
# Load commands
        time.sleep(2)

# Arm vehicle
        self.armed = True



#world_stat = GazeboWorldStats("localhost", 11345)
#world_stat.get_sim_time(start_sim_time_callback)

# monitor mission execution
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
        #data.show()

if __name__ == "__main__":
    connection_string = '127.0.0.1:14540'
    # Connect to the Vehicle
    print "Connecting"
    v = connect(connection_string, wait_ready=True, vehicle_class=FastVehicle)

    #pilot = FastVehicle()
    v.fly()

