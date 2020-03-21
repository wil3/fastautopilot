
import unittest
import sys
sys.path.append("..")
from planner import *

class TestEvolver(unittest.TestCase):
    """
    def test_quaternion_to_euler(self):
        w, x, y, z = ()
        quaternion_to_eulerian(w, x, y, z)

    """
    def test_euler_to_quaternion(self):
        roll, pitch, yaw = (0, 0, 0)
        #w, x, y, z = eulerian_to_quaternion(roll, pitch, yaw)
        actual = eulerian_to_quaternion(roll, pitch, yaw)

        expected = [1, 0, 0, 0]
        self.assertSequenceEqual(expected, actual)
        
    def test_euler_to_quaternion_45(self):
        roll, pitch, yaw = (45, 45, 45)
        #w, x, y, z = eulerian_to_quaternion(roll, pitch, yaw)
        actual = eulerian_to_quaternion(roll, pitch, yaw)

        print actual
        expected = [1, 0, 0, 0]
        self.assertSequenceEqual(expected, actual)

if __name__ == "__main__":
    unittest.main()
