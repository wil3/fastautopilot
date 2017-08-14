
from dronekit import connect, Command, LocationGlobal
from pymavlink import mavutil
import time, sys, argparse, math

import multiprocessing as mp

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
import mpl_toolkits.mplot3d.axes3d as p3


class MyFuncAnimation(animation.FuncAnimation):
    """
    Unfortunately, it seems that the _blit_clear method of the Animation
    class contains an error in several matplotlib verions
    That's why, I fork it here and insert the latest git version of
    the function.
    """
    def _blit_clear(self, artists, bg_cache):
        # Get a list of the axes that need clearing from the artists that
        # have been drawn. Grab the appropriate saved background from the
        # cache and restore.
        axes = set(a.axes for a in artists)
        for a in axes:
            if a in bg_cache: # this is the previously missing line
                a.figure.canvas.restore_region(bg_cache[a])


connection_string       = '127.0.0.1:14540'
MAV_MODE_AUTO   = 4

# Connect to the Vehicle
print "Connecting"
vehicle = connect(connection_string, wait_ready=True)

manager = mp.Manager()
lock = mp.Lock()




lat = manager.list()
lon = manager.list()
alt = manager.list()

loc = manager.list()


def init_tracker(l, _lat, _lon, _alt):
    print "Init tracker"
    #init home values
    arr_lat = np.asarray(_lat) 
    arr_lon = np.asarray(_lon) 
    arr_alt = np.asarray(_alt)
    fig = plt.figure()

    ax = fig.gca(projection='3d')
    ax.set_xlabel("Latitude")
    ax.set_ylabel("Longitude")
    ax.set_zlabel("Altitude")
    """
    ax.set_xlim3d([0.0, 50.0])
    ax.set_ylim3d([0.0, 10.0])
    ax.set_zlim3d([0.0, 15.0])
    """

    print "Lat len", len(arr_lat)
    print "Lon len", len(arr_lon)
    print "Alt len", len(arr_alt)
    line, = ax.plot(arr_lat, arr_lon, arr_alt)

    def update_tracker(num, __lat, __lon, __alt):
        try:
            l.acquire()
            arr_lat = np.asarray(__lat) 
            arr_lon = np.asarray(__lon) 
            arr_alt = np.asarray(__alt)
            # Resize 
            ax.set_xlim3d([min(arr_lat), max(arr_lat)])
            ax.set_ylim3d([min(arr_lon), max(arr_lon)])
            ax.set_zlim3d([min(arr_alt), max(arr_alt)])
            ax.figure.canvas.draw()

            line.set_data(arr_lat, arr_lon)
            line.set_3d_properties(arr_alt)
            l.release()
        except Exception as e:
            print "E1", e
        return line,

    line_ani =MyFuncAnimation(fig, update_tracker, 25, fargs=(_lat, _lon, _alt),  interval=50, blit=False)
    plt.show()



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
    if (len(lat) > 0 and lat[-1] == curr_lat and 
       len(lon) > 0 and lon[-1] == curr_lon and 
       len(alt) > 0 and alt[-1] == curr_alt):
        return 

    print "Lat=", curr_lat, " Lon=", curr_lon, " Alt=", curr_alt
    lock.acquire()
    lat.append(curr_lat)
    lon.append(curr_lon)
    alt.append(curr_alt)
    lock.release()

dt_thrust = 0
dt_yaw = 0
dt_roll = 0
dt_pitch = 0

@vehicle.on_message('ATTITUDE_TARGET')
def attitude_target_listener(self, name, target):
    pass
    #print "Target: ", target
################################################################################################
# Start mission example
################################################################################################

# wait for a home position lock
while not home_position_set:
    print "Waiting for home position..."
    time.sleep(1)



#Init position
location = vehicle.location
lat.append(location.global_relative_frame.lat)
lon.append(location.global_relative_frame.lon)
alt.append(location.global_relative_frame.alt)
p = mp.Process(target=init_tracker, args=(lock, lat, lon, alt, ))
p.start() 

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

home = vehicle.location.global_relative_frame

# takeoff to 10 meters
wp = get_location_offset_meters(home, 0, 0, 10);
cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, wp.alt)
cmds.add(cmd)

# move 10 meters north
wp = get_location_offset_meters(wp, 3, 0, 0);
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
time.sleep(1)



# Close vehicle object before exiting script
vehicle.close()
time.sleep(1)

p.join()
