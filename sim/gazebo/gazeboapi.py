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

def _get_manager():
    yield From(pygazebo.connect(('localhost', 11345)))


@trollius.coroutine
def _publish_loop():
    manager = yield From(pygazebo.connect(('localhost', 11345)))
    publisher = yield From(manager.advertise('/gazebo/default/world_control', 'gazebo.msgs.WorldControl'))
    world = pygazebo.msg.world_control_pb2.WorldControl()
    world.reset.model_only = True 

    #for i in range(2):
    while True:
        print("Publish message")
        yield From(publisher.publish(world))
        yield From(trollius.sleep(2.0))

def reset():
    loop = trollius.get_event_loop()
    loop.run_until_complete(_publish_loop())

def _world_stats_callback(data):
    world_stats = pygazebo.msg.world_stats_pb2.WorldStatistics()
    world_stats.ParseFromString(data)
    print "Time  ", world_stats.real_time

def _world_stats_subscribe_loop():
    manager = yield From(pygazebo.connect(('localhost', 11345)))
    subscriber = manager.subscribe('/gazebo/default/world_stats', 'gazebo.msgs.WorldStatistics', _position_callback)
    while True:
        yield From(subscriber.wait_for_connection())

def world_stats():
    loop = trollius.get_event_loop()
    loop.run_until_complete(_world_stats_subscribe_loop())

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

def _world_reset_message():
    world = pygazebo.msg.world_control_pb2.WorldControl()
    world.reset.model_only = True 
    return world


TOL = 0.1
subscriber = None

def at_origin(pose):
    if math.fabs(pose.position.x) < TOL and math.fabs(pose.position.y) < TOL:
        return True
    else:
        return False

def _reset_callback(data):
    try:
        msg = pygazebo.msg.poses_stamped_pb2.PosesStamped()
        msg.ParseFromString(data)
        for pose in msg.pose:
            print pose
            if pose.name == "iris":
                if at_origin(pose):
                    waiting_for_reset = False
                    print "RESET!"
                    subscriber.remove()
                break
    except Exception as e:
        print e

def _reset_model():
    
    #start listening for the event
    manager = yield From(pygazebo.connect(('localhost', 11345)))
    publisher = yield From(manager.advertise('/gazebo/default/world_control', 'gazebo.msgs.WorldControl'))
    subscriber = manager.subscribe('/gazebo/default/pose/info', 'gazebo.msgs.PosesStamped', _reset_callback)
    world = _world_reset_message()
    #while waiting_for_reset:
    #yield From(publisher.publish(world))
    yield From(trollius.sleep(5))
    print "Looking for reset message"
    yield From(subscriber.wait_for_connection())

def reset_model():
    loop = trollius.get_event_loop()
    loop.run_until_complete(_reset_model())#_world_control_subscribe_loop())

try:
    reset_model()
except Exception as e:
    print e

