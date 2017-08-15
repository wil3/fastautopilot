from sim.gazebo.gazeboapi import GazeboWorldStats

from dronekit import connect, Command, LocationGlobal
from pymavlink import mavutil
import time, sys, argparse, math

#import multiprocessing as mp

from tracker import *



connection_string       = '127.0.0.1:14540'
MAV_MODE_AUTO   = 4
START_TIME = 0
END_TIME = 0

# Connect to the Vehicle
print "Connecting"
vehicle = connect(connection_string, wait_ready=True)

#manager = mp.Manager()
#lock = mp.Lock()
#lock_att = mp.Lock()


"""
# Keep track of total trajectory
# To plot need them as separate lists
lat = manager.list()
lon = manager.list()
alt = manager.list()

#is is better ot keep a single time? or for each?
att_time = manager.list()
thrust = manager.list()
yaw = manager.list()
roll = manager.list()
pitch = manager.list()

q_loc = mp.Queue()
q_att = mp.Queue()
"""
list_loc = []
list_att = []




def PX4setMode(mavMode):
    vehicle._master.mav.command_long_send(vehicle._master.target_system, vehicle._master.target_component,
                                               mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                                               mavMode,
                                               0, 0, 0, 0, 0, 0)



def get_location_offset_meters(original_location, dNorth, dEast, alt):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the
    specified `original_location`. The returned Location adds the entered `alt` value to the altitude of the `original_location`.
    The function is useful when you want to move the vehicle around specifying locations relative to
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return LocationGlobal(newlat, newlon,original_location.alt+alt)





################################################################################################
# Listeners
################################################################################################

home_position_set = False

#Create a message listener for home position fix
@vehicle.on_message('HOME_POSITION')
def listener(self, name, home_position):
    global home_position_set
    home_position_set = True

@vehicle.on_attribute('attitude')
def attitude_listener(self, name, attitude):
    #print "yaw={} pitch={} roll={}".format(attitude.yaw, attitude.pitch, attitude.roll)
    #print "gps={}".format(vehicle.gps_0)
    pass

@vehicle.on_attribute('location')
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

#    print "Lat=", curr_lat, " Lon=", curr_lon, " Alt=", curr_alt
    if vehicle.armed:
        list_loc.append(location.global_relative_frame)
        """
        q_loc.put(location.global_relative_frame)
        lock.acquire()
        lat.append(curr_lat)
        lon.append(curr_lon)
        alt.append(curr_alt)
        lock.release()
        """

time_boot = -1 

@vehicle.on_message('ATTITUDE_TARGET')
def attitude_target_listener(self, name, target):
    global time_boot 


    """
    if (len(roll) > 0 and roll[-1] == target.body_roll_rate and
        len(pitch) > 0 and pitch[-1] == target.body_pitch_rate and
        len(yaw) > 0 and yaw[-1] == target.body_yaw_rate and
        len(thrust) > 0 and thrust[-1] == target.thrust):
        return
    """

    if vehicle.armed:
        if time_boot < 0:
            time_boot = target.time_boot_ms
        list_att.append(target)
        """
        q_att.put(target)
        lock_att.acquire()
        att_time.append(target.time_boot_ms - time_boot) 
        roll.append(target.body_roll_rate)
        pitch.append(target.body_pitch_rate)
        yaw.append(target.body_yaw_rate)
        thrust.append(target.thrust)
        lock_att.release()
        """
################################################################################################
# Start mission example
################################################################################################

# wait for a home position lock
while not home_position_set:
    print "Waiting for home position..."
    time.sleep(1)



#Init position
"""
location = vehicle.location
lat.append(location.global_relative_frame.lat)
lon.append(location.global_relative_frame.lon)
alt.append(location.global_relative_frame.alt)
"""
#p = Tracker(q_loc, q_att)#mp.Process(target=init_tracker, args=(q_loc, q_att))#lock,lock_att, lat, lon, alt, att_time,  thrust, yaw, pitch, roll))
#p.start() 

# Display basic vehicle state
print " Type: %s" % vehicle._vehicle_type
print " Armed: %s" % vehicle.armed
print " System status: %s" % vehicle.system_status.state
print " GPS: %s" % vehicle.gps_0
print " Alt: %s" % vehicle.location.global_relative_frame.alt

# Change to AUTO mode
PX4setMode(MAV_MODE_AUTO)
time.sleep(1)

# Load commands
cmds = vehicle.commands
cmds.clear()

wp_loc = []
home = vehicle.location.global_relative_frame
wp_loc.append(home)
# takeoff to 10 meters
wp = get_location_offset_meters(home, 0, 0, 10);
wp_loc.append(wp)
cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, wp.alt)
cmds.add(cmd)

# move 10 meters north
wp = get_location_offset_meters(wp, 3, 0, 0);
wp_loc.append(wp)
cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, wp.alt)
cmds.add(cmd)

# land
wp = get_location_offset_meters(home, 0, 0, 10);
cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, wp.alt)
cmds.add(cmd)


# Upload mission
cmds.upload()
time.sleep(2)

# Arm vehicle
vehicle.armed = True


def start_sim_time_callback(sim_time):
    print "Start Sim Time", sim_time.sec
    START_TIME = sim_time.sec
    
def end_sim_time_callback(sim_time):
    print "End Sim Time", sim_time.sec
    END_TIME = sim_time.sec
    print "Lapse time ", END_TIME - START_TIME

#world_stat = GazeboWorldStats("localhost", 11345)
#world_stat.get_sim_time(start_sim_time_callback)

# monitor mission execution
nextwaypoint = vehicle.commands.next
while nextwaypoint < len(vehicle.commands):
    if vehicle.commands.next > nextwaypoint:
        display_seq = vehicle.commands.next+1
        print "Moving to waypoint %s" % display_seq
        nextwaypoint = vehicle.commands.next
    time.sleep(1)

# wait for the vehicle to land
while vehicle.commands.next > 0:
    time.sleep(1)


# Disarm vehicle
vehicle.armed = False

#world_stat.get_sim_time(end_sim_time_callback)

time.sleep(1)



# Close vehicle object before exiting script
vehicle.close()
time.sleep(1)

print "Vehicle closed"
#p.join()

data = FlightData()
data.trajectory(list_loc,wp_loc)
data.inputs(list_att)
data.show()
