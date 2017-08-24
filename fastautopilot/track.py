import abc 
import numpy as np
import matplotlib.pyplot as plt
from collections import deque

class Gate(object):
    """ A waypoint defines how a MAV must fly through it """
    def __init__(self):

        """ Best time this gate has had """
        self.time_best = float('inf')

    @abc.abstractmethod
    def detected(self, x, y, z):  
        """ True if detected this given 3D point is passed the gate """

    @abc.abstractmethod
    def plot2d(self, ax):
        """ How the gate should be plotted in 2d """

    @abc.abstractmethod
    def plot3d(self, ax):
        """ How the gate should be plotted in 3d """


class SimplePointGate(Gate):
    def __init__(self, x, y, z, radius=0):
        super(SimplePointGate, self).__init__()
        self.x = x
        self.y = y
        self.z = z
        self.radius = radius

        (self.enu_x , self.enu_y, self.enu_z) = self.NEDtoENUBodyFrame(self.x, self.y, self.z)

    def detected(self, x, y, z):  
        return (self._d([self.x, self.y, self.z], [x, y, z]) <= self.radius/2.0)

    def _d(self, pt1, pt2):
        """
        Return distance between two points
        """
        a = np.array(pt1)
        b = np.array(pt2)
        d = np.linalg.norm(a-b)
        return d

    def plot3d(self, ax):
        

        scale = self.radius/2.0
        
        u = np.linspace(0, 2 * np.pi, 10)
        v = np.linspace(0, np.pi, 10)
        x = scale * np.outer(np.cos(u), np.sin(v)) + self.enu_x
        y = scale * np.outer(np.sin(u), np.sin(v)) + self.enu_y
        z = scale * np.outer(np.ones(np.size(u)), np.cos(v)) + self.enu_z

        ax.plot_wireframe(x, y, z, color='r')

    def plot2d(self, ax):
        c = plt.Circle((self.enu_x, self.enu_y), self.radius/2.0, color='r', fill=False)
        ax.add_artist(c)


    def NEDtoENUBodyFrame(self, x, y, z):
        return (x, -y, -z)

    def __str__(self):
        return "x={}, y={}, z={} Best={}".format(self.x, self.y, self.z, self.time_best)

    def __repr__(self):
        return self.__str__()

class Track:
    """ A track is made up of a sequence of waypoints. Each waypoint is a constraint """
    def __init__(self, gates=None):

        if gates:
            self.gates = gates
        else:
            self.gates = [] 

        self.time_best = None

        self.gate_count = len(gates)

    def add_gate(self, gate):
        self.gates.append(gate)
        self.gate_count(len(gates))

    def __str__(self):
        return str(self.gates)

    def __repr__(self):
        return self.__str__()


def straight_line_track(num_gates = 1, gate_spacing = 5, altitude = -5):
    gates =  [] 
    for i in range(num_gates):
        gates.append(SimplePointGate(i*gate_spacing, 0, altitude, radius = 1))
    return Track(gates)
"""
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
"""
