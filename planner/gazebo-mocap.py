import time
import pygazebo
import pygazebo.msg.poses_stamped_pb2

# There is no Python 3 support for drone kit so use trollius
# WARNING: trollius is depreciated
import trollius
from trollius import From
import trollius as asyncio

from dronekit import connect, Command, LocationGlobal, Vehicle
from pymavlink import mavutil
import time, sys, argparse, math

class MOCAP(object):
    """
    PX4 will timeout with "mocap timeout" if there is not a message
    sent at least ever 0.2 seconds see src/modules/local_position_estimator/sensors/mocap.cpp
    Look at this discussion for frame conversions
    https://github.com/mavlink/mavros/issues/216
    """
    def __init__(self, px4_connection_string):
        self.vehicle = connect(px4_connection_string, wait_ready=True)
#        self.vehicle._master.mav.set_send_callback(self.send_callback)

    def ENUtoNEDBodyFrame(self, x, y, z):
        return (x, -y, -z)

    def send_callback(self, msg):
        print "Message ", msg


    def _send_att_pos_mocap(self, q, x, y, z):
        """
        http://mavlink.org/messages/common#ATT_POS_MOCAP
        http://osrf-distributions.s3.amazonaws.com/gazebo/msg-api/7.1.0/poses__stamped_8proto.html
        """
        time_usec = int(round(time.time() * 1000000))
        #print "Timestamp ", time_usec
        self.vehicle._master.mav.att_pos_mocap_send(time_usec, q, x, y, z)
        #print "q={} {} x={} y={} z={}".format(q, x, y, z)

    def _poses_callback(self, data):
        msg = pygazebo.msg.poses_stamped_pb2.PosesStamped()
        msg.ParseFromString(data)
        for pose in msg.pose:
            if pose.name == "iris":
                pos = pose.position 
                o = pose.orientation
                (ned_x, ned_y, ned_z) = o_ned = self.ENUtoNEDBodyFrame(o.x, o.y, o.z)
                q = [o.w, ned_x, ned_y, ned_z]
                (ned_pos_x, ned_pos_y, ned_pos_z) = self.ENUtoNEDBodyFrame(pos.x, pos.y, pos.z)
                #print "q={} {} x={} y={} z={}".format(pose.name,q, pos.x, pos.y, pos.z)
                self._send_att_pos_mocap(q, ned_pos_x, ned_pos_y, ned_pos_z)
        #time.sleep(0.5)

    def _get_poses(self):
        
        #start listening for the event
        manager = yield From(pygazebo.connect(('localhost', 11345)))
        subscriber = manager.subscribe('/gazebo/default/pose/info', 'gazebo.msgs.PosesStamped', self._poses_callback)
        while True:
            yield From(subscriber.wait_for_connection())

    def start(self):

        loop = trollius.get_event_loop()
        loop.run_until_complete(self._get_poses())


if __name__ == "__main__":
    px4_connection_string = '127.0.0.1:14540'
    gen = MOCAP(px4_connection_string)
    gen.start()
