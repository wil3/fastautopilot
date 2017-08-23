from multiprocessing import Process, Queue

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

class ControlInputs(object):
    def __init__(self, t, roll, pitch, yaw, thrust):
        self.t = t
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.thrust = thrust

class FlightData(object):

    def __init__(self):
        self.lat = []
        self.lon = [] 
        self.alt = []

        self.yaw = []
        self.thrust = []
        self.roll = []
        self.pitch = []
        self.t = []

    def _split_trajectory_data(self, data):
        for loc in data:
            (x, y, z) = self.NEDtoENUBodyFrame(loc.x, loc.y, loc.z)
            self.lat.append(x)
            self.lon.append(y)
            self.alt.append(z)

    def NEDtoENUBodyFrame(self, x, y, z):
        return (x, -y, -z)

    def _split_waypoint_data(self, data):
        lat = []
        lon = []
        alt = []
        for wp in data:
            #Convert back to ENU
            (x, y, z) = self.NEDtoENUBodyFrame(wp[0], wp[1], wp[2])

            lat.append(x)
            lon.append(y)
            alt.append(z)
        return (lat, lon, alt)


    def _split_input_data(self, data):
        """
        return tuple of time and AETR values
        """
        t = []
        aileron = []
        elevator = []
        rudder = []
        throttle = []

        for input in data:
            self.roll.append(input.body_roll_rate)
            self.pitch.append(input.body_pitch_rate)
            self.yaw.append(input.body_yaw_rate)
            self.thrust.append(input.thrust)
            self.t.append(input.time_boot_ms)

        return (t, aileron, elevator, throttle, rudder)



    def trajectory(self, data, wp):
        self._split_trajectory_data(data)
        (wp_lat, wp_lon, wp_alt) = self._split_waypoint_data(wp)
        self._plot_trajectory(wp_lat, wp_lon, wp_alt)

        self._plot_trajectory_2D(self.lat, self.lon, wp_lat, wp_lon)

    def inputs(self, data):
        self._split_input_data(data)
        self._plot_input()

    def identify_control_points(self, t, input):
        """ 
        Condense inputs to only those that matter, i.e., when a change
        actually occurs
        """
        pts = []
        pts.append( (t[0], input[0]) )
        prev = input[0]
        for i in range(1, len(input)):
            # If we changed since the last input then 
            if prev != input[i]:
                pts.append( (t[i], input[i]) )
            prev = input[i]

        return pts
            


    def _plot_input(self):
#        fig = plt.figure()
        f, (ax1, ax2, ax3, ax4) = plt.subplots(4, sharex=True, sharey=True)
        #ax1 = fig.add_subplot(2,1,1)#plt.subplots()
        #ax1.set_xlabel("Time")
        ax1.set_ylabel("Thrust")
        #ax1.set_ylim([0, 1.2])
        ax1.plot(self.t, self.thrust)
        ax1.plot(self.t[1:], np.diff(self.thrust), 'r--')

        ax2.plot(self.t, self.pitch)
        ax2.set_ylabel("Pitch")

        #ax2 = fig.add_subplot(2,1,2)#plt.subplots()
        #ax2.set_xlabel("Time")
        ax3.set_ylabel("Yaw")
        #ax1.set_ylim([0, 1.2])
        ax3.plot(self.t, self.yaw)

        ax4.plot(self.t, self.roll)
        ax4.set_ylabel("Roll")
        ax4.set_xlabel("Time")

        plt.setp([a.get_xticklabels() for a in f.axes[:-1]], visible=False)


    def _plot_trajectory_2D(self, x, y, wp_x, wp_y):
        f, (ax1) = plt.subplots(1)
        ax1.set_ylabel("Y (m)")
        ax1.set_xlabel("X (m)")
        ax1.plot(x, y, label="Flight path")
        ax1.plot(wp_x, wp_y, 'ro', label="Waypoints")
        ax1.set_title("Birds Eye View of Flight Path")
        ax1.legend()

    def _plot_trajectory(self, wp_lat, wp_lon, wp_alt):
        print wp_lat
        print wp_lon
        print wp_alt
        fig = plt.figure()

        ax = fig.add_subplot(111, projection='3d')
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_zlabel("Z (m)")
        ax.set_xlim3d([min(self.lat), max(self.lat)])
        ax.set_ylim3d([min(self.lon), max(self.lon)])
        ax.set_zlim3d([min(self.alt), max(self.alt)])
        plt.ticklabel_format(style='plain', axis='both')


        line, = ax.plot(self.lat, self.lon, zs=self.alt)
        ax.plot(wp_lat, wp_lon, 'ro', zs=wp_alt)

    def show(self):
        plt.show()

class Tracker(Process):
    def __init__(self, q_loc, q_att):
        super(Tracker, self).__init__()
        self.q_loc = q_loc
        self.q_att = q_att

        self.lat = []
        self.lon = [] 
        self.alt = []
#t, _thrust, _yaw, _pitch, _roll
    def update_tracker(self, num, line, ax):# l,  __lat, __lon, __alt):
        try:
            for i in range(100):
                loc = self.q_loc.get()
                """
                print "Update tracker"
                #while q_loc.full():
                print "Adding ", loc
                print loc.lat
                print loc.lon
                print loc.alt
                print type(loc.lat)
                print type(loc.lon)
                print type(loc.alt)
                """
                self.lat.append(loc.lat)
                self.lon.append(loc.lon)
                self.alt.append(loc.alt)
        except:
            pass
#        print "update"
        #l.acquire()
        #lat = np.asarray(__lat) 
        #lon = np.asarray(__lon) 
        #alt = np.asarray(__alt)
        # Resize 
        #print "Lat=", type(lat), " ", len(lat)
        #print "Lon=", type(lon), " ", len(lon)
        #print "Alt=", type(alt), " ", len(alt)


        """
        _lat = np.array(self.lat)
        _lon = np.array(self.lon)
        _alt = np.array(self.alt)
        print _lat
        print _lon
        print _alt
        _lat = self.lat
        _lon = self.lon
        _alt = self.alt
        """

        ax.set_xlim3d([min(self.lat), max(self.lat)])
        ax.set_ylim3d([min(self.lon), max(self.lon)])
        ax.set_zlim3d([min(self.alt), max(self.alt)])
        ax.figure.canvas.draw()

        try:
            line.set_data(self.lat, self.lon)
            line.set_3d_properties(self.alt)
            #l.release()
        except Exception as e:
            print "E1", e
        return line,


    def update_att(self, num, l, lines, extras):#, _yaw, _pitch, _roll ):

        try:
            for i in range(0,len(lines)):
                x = extras[i]["x"]
                y = extras[i]["y"]
                ax = extras[i]["ax"]
                print "Start"
                if len(x) == 0:
                    return

                l.acquire()
                
                arr_x = np.asarray(x)
                arr_y= np.asarray(y)
                ax.set_xlim([0, max(arr_x)])
                
                lines[i].set_data(x,y)
                l.release()
                print "End"
                print "X: ", len(x)
                print "Y: ", len(y)
        except Exception as e:
            print "E2", e

        return lines,



    def run(self):#l,lock_att, _lat, _lon, _alt, _t, _thrust, _yaw, _pitch, _roll):
        print "Init tracker"
        #init home values

#    track_trajectory(l, _lat, _lon, _alt)
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.set_xlabel("Latitude")
        ax.set_ylabel("Longitude")
        ax.set_zlabel("Altitude")

        #3D plots cant be initialized with empty data so populate
        """
        loc = self.q_loc.get()
        self.lat.append(loc.lat)
        self.lon.append(loc.lon)
        self.alt.append(loc.alt)
        arr_lat = np.asarray(loc.lat) 
        arr_lon = np.asarray(loc.lon) 
        arr_alt = np.asarray(loc.alt)
        
        print "Lat=", type(lat), " ", len(lat)
        print "Lon=", type(lon), " ", len(lon)
        print "Alt=", type(alt), " ", len(alt)
        """

        #line, = ax.plot(lat, lon, zs=alt)
        line, = ax.plot([], [], zs=[])


        animi = []
        #animi.append(MyFuncAnimation(fig, update_tracker, 25, fargs=(line, ax, l,  _lat, _lon, _alt),  interval=50, blit=False))
        animi.append(MyFuncAnimation(fig, self.update_tracker, frames=25, fargs=(line, ax),  interval=50, blit=False))

        """
        lines = []
        data = []

        att_fig = plt.figure()
        ax1 = att_fig.add_subplot(2,1,1)#plt.subplots()
        ax1.set_xlabel("Time")
        ax1.set_ylabel("Thrust")
        ax1.set_ylim([0, 1.2])
        thrust_line, = ax1.plot([], [])

        lines.append(thrust_line)
        data.append( {"ax": ax1, "x": _t, "y": _thrust})

        ax2 = att_fig.add_subplot(2,1,2)#plt.subplots()
        ax2.set_xlabel("Time")
        ax2.set_ylabel("Yaw")
        ax2.set_ylim([-2*3.14, 2*3.14])
        yaw_line, = ax2.plot([], [])

        #lines.append(yaw_line)
        #data.append( {"ax": ax2, "x": _t, "y": _yaw})

        #animi.append(MyFuncAnimation(att_fig, update_att, 25, fargs=(thrust_line, yaw_line, ax1, lock_att, _t,_thrust ),  interval=50, blit=False))
        animi.append(MyFuncAnimation(att_fig, update_att, 25, fargs=(lock_att, lines, data ),  interval=50, blit=False))
        """
#    try:
        plt.show()
#    except AttributeError: pass



