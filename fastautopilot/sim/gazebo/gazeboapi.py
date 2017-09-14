import math
import pygazebo
import pygazebo.msg.world_control_pb2
import pygazebo.msg.model_pb2
import pygazebo.msg.world_stats_pb2
import pygazebo.msg.pose_pb2
import pygazebo.msg.pose_stamped_pb2
import pygazebo.msg.poses_stamped_pb2
import pygazebo.msg.model_pb2
import pygazebo.msg.gps_pb2

# There is no Python 3 support for drone kit so use trollius
# WARNING: trollius is depreciated
import trollius
from trollius import From
import trollius as asyncio

import traceback

import concurrent

class GazeboAPI:
    MODEL_NAME = "iris"
    ORIGIN_TOL = 0.1
    TOPIC = '/gazebo/default/world_stats'
    TYPE = 'gazebo.msgs.WorldStatistics'

    def __init__(self, host, port):
        self.host = host
        self.port = port

        self.pose_info_subscriber = None
        self.waiting_for_message = True

    def get_sim_time(self, callback):
        self.callback = callback
        loop = trollius.get_event_loop()
        loop.run_until_complete(self._world_stats_subscribe_loop())

    """
    def get_world_stats_once(self, callback):
        loop = trollius.get_event_loop()
        loop.run_until_complete(_world_stats_subscribe_loop())
    """

    def _world_stats_subscribe_loop(self):
        #manager = yield From(self._manager()) 
        manager = yield From(pygazebo.connect((self.host, self.port)))
        subscriber = manager.subscribe(self.TOPIC, self.TYPE , self._world_stats_callback)
        self.waiting = True 
        while self.waiting:
            yield From(subscriber.wait_for_connection())

    def _world_stats_callback(self, data):
        world_stats = pygazebo.msg.world_stats_pb2.WorldStatistics()
        world_stats.ParseFromString(data)
        #print "Time  ", world_stats.real_time
        if self.waiting:
            self.callback(world_stats.sim_time)
        self.waiting = False

    def _world_reset_message(self):
        world = pygazebo.msg.world_control_pb2.WorldControl()
        world.reset.model_only = True 
        return world

    # TODO Replace with common distance function
    # FIXME Not checking z, shouldnt happen because we've landed but still
    def _at_origin(self, pose):
        if math.fabs(pose.position.x) < self.ORIGIN_TOL and math.fabs(pose.position.y) < self.ORIGIN_TOL:
            return True
        else:
            return False


    def close_connection(self):
        for conn in self.pose_info_subscriber._connections:
            print "Closing connection"
            conn.socket.close()

    def _reset_callback(self, data):
        msg = pygazebo.msg.poses_stamped_pb2.PosesStamped()
        msg.ParseFromString(data)
        for pose in msg.pose:
            if pose.name == self.MODEL_NAME:
                if self._at_origin(pose):
                    self.waiting_for_message = False
                    #print "Model reset!"
                break

    def _reset_model(self):#, future):
        
        #start listening for the event
        #print "connect"
        manager = yield From(pygazebo.connect((self.host, self.port)))
        #print "pub init"
        publisher = yield From(manager.advertise('/gazebo/default/world_control', 'gazebo.msgs.WorldControl'))
        #print "sub init"
        self.pose_info_subscriber = manager.subscribe('/gazebo/default/pose/info', 'gazebo.msgs.PosesStamped', self._reset_callback)
        world = self._world_reset_message()
        self.waiting_for_message = True

        publish_interval = 1 # publish at this interval
        poll_interval = 0.1 # Sleep for this long to wait for a response message
        last_time = 0
        while self.waiting_for_message: 

            #print "sleep"
            yield From(trollius.sleep(poll_interval))
            #print "pub"
            dt = time.time() - last_time
            if dt > publish_interval: 
                yield From(publisher.publish(world))
                last_time = time.time()
            #print "wait"
            #yield From(self.pose_info_subscriber.wait_for_connection())

        """
        yield From(publisher.publish(world))
        yield From(self.pose_info_subscriber.wait_for_connection())

        while self.waiting_for_message: 
            print "waiting"
            yield From(trollius.sleep(0.5))

        """

        #self.pose_info_subscriber.remove()
        #self.reset_model_callback()
        #future.set_result('Done!')

    def reset_model(self, callback):
        #self.reset_model_callback = callback
        self.loop = asyncio.get_event_loop()
        #future = asyncio.Future()
        #asyncio.ensure_future(self._reset_model(future))
        #loop.run_until_complete(future)
        #print(future.result())

        #executor = concurrent.futures.ThreadPoolExecutor(5)
        #loop.set_default_executor(executor)


        self.loop.run_until_complete(self._reset_model())
        print "Loop complete, closing connection"
        # This causes Bad file descriptor'
        self.close_connection()
        #executor.shutdown(wait=True)

        #Closing the event  causes an exception to be raised, this is irriversible do not do this!
        #self.loop.close()
        callback()

@trollius.coroutine
def _publish_loop():
    manager = yield From(pygazebo.connect(('localhost', 11345)))
    publisher = yield From(manager.advertise('/gazebo/default/world_control', 'gazebo.msgs.WorldControl'))
    world = pygazebo.msg.world_control_pb2.WorldControl()
    world.reset.model_only = True 

    while True:
        print("Publish message")
        yield From(publisher.publish(world))
        yield From(trollius.sleep(2.0))

def reset():
    loop = trollius.get_event_loop()
    loop.run_until_complete(_publish_loop())


def _position_callback(data):
    try:
        message = pygazebo.msg.model_pb2.Model()
        message.ParseFromString(data)
        print "Msg  ", message 
        waiting_for_reset = False
    except Exception as e:
        print e

def _position_subscribe_loop():
    manager = yield From(pygazebo.connect(('localhost', 11345)))
    subscriber = manager.subscribe('/gazebo/default/model/info', 'gazebo.msgs.Model', _position_callback)
    while True:
        yield From(subscriber.wait_for_connection())

def position():
    loop = trollius.get_event_loop()
    loop.run_until_complete(_position_subscribe_loop())



def _world_control_callback(data):
    message = pygazebo.msg.model_pb2.WorldControl()
    message.ParseFromString(data)
    print "Msg  ", message 

def _world_control_subscribe_loop2():
    manager = yield From(pygazebo.connect(('localhost', 11345)))
    subscriber = manager.subscribe('/gazebo/default/world_control', 'gazebo.msgs.WorldControl', _position_callback)
    while True:
        yield From(subscriber.wait_for_connection())

def world_control():
    loop = trollius.get_event_loop()
    loop.run_until_complete(_world_control_subscribe_loop())



waiting_for_reset = True

def _world_control_subscribe_loop(manager):
    subscriber = manager.subscribe('/gazebo/default/world_control', 'gazebo.msgs.WorldControl', _position_callback)
    while waiting_for_reset:
        yield From(subscriber.wait_for_connection())
        yield From(trollius.sleep(0.5))

def _world_control_reset_publish(manager):
    publisher = yield From(manager.advertise('/gazebo/default/world_control', 'gazebo.msgs.WorldControl'))
    print "got publisher"
    world = pygazebo.msg.world_control_pb2.WorldControl()
    world.reset.model_only = True 

    while waiting_for_reset:
        print("Publish message")
        yield From(publisher.publish(world))
        yield From(trollius.sleep(0.5))



class TestReset:
    def reset_callback(self):
        print "Model reset"

    def reset_test(self):
        api = GazeboAPI("localhost", 11345)
        api.reset_model(self.reset_callback)

if __name__ == "__main__":
    """
    world_stat = GazeboWorldStats("localhost", 11345)
    def sim_time_callback(sim_time):
        print "Start Sim Time", sim_time.sec
    world_stat.get_sim_time(sim_time_callback)
    """
    TestReset().reset_test()
