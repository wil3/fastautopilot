import os
import logging.config
import yaml
import unittest
import sys
sys.path.append("..")
import time
from sim.gazebo.gazeboapi import GazeboAPI 

class TestGazeboAPI(unittest.TestCase):
    """
    def test_reset(self):
        gazebo_host="127.0.0.1"
        gazebo_port=11345
        def reset_callback():
            print "Reset"
        gz = GazeboAPI(gazebo_host, gazebo_port)
        gz.listen(reset_callback)
        time.sleep(1)
        gz.reset()
        time.sleep(1)
        gz.shutdown()
    """
    def test_reset(self):
        gazebo_host="127.0.0.1"
        gazebo_port=11345
        def reset_callback():
            print "Reset"
        gz = GazeboAPI(gazebo_host, gazebo_port)
        gz.listen(reset_callback)
        time.sleep(1)
        for i in range(1000):
            gz.reset()
        time.sleep(1)
        gz.shutdown()

    """
    def test_reset(self):
        gazebo_host="127.0.0.1"
        gazebo_port=11345
        def reset_callback():
            print "Reset"
        GazeboAPI(gazebo_host, gazebo_port).reset_model(reset_callback)
    def test_reset_multiple(self):
        #
        # There is a bug causing crash to many files
        gazebo_host="127.0.0.1"
        gazebo_port=11345
        
        self.counter = -1 
        def reset_callback():
            print "Reset ", self.counter
            self.counter += 1

        for i in range(1000):
            GazeboAPI(gazebo_host, gazebo_port).reset_model(reset_callback)
            while i != self.counter:
                time.sleep(1)

"""

    """
    def test_sim_time(self):
        world_stat = GazeboAPI("localhost", 11345)
        def sim_time_callback(sim_time):
            print "Start Sim Time", sim_time.sec
        world_stat.get_sim_time(sim_time_callback)
    """
def init_logging():
    log_config = os.path.join(".", "../../conf/logging.yaml")
    if os.path.exists(log_config):
        with open(log_config, 'rt') as f:
            log_config = yaml.load(f.read())
        logging.config.dictConfig(log_config)
    else:
        raise Exception("Cannot find log configuration file")



if __name__ == "__main__":
    init_logging()
    unittest.main()
