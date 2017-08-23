
import unittest
import sys
sys.path.append("..")
from tracker import FlightData

class TestGazeboAPI(unittest.TestCase):

    def test_reset(self):
        pass

    def test_sim_time(self):
        world_stat = GazeboAPI("localhost", 11345)
        def sim_time_callback(sim_time):
            print "Start Sim Time", sim_time.sec
        world_stat.get_sim_time(sim_time_callback)

if __name__ == "__main__":
    unittest.main()
