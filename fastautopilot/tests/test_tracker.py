import unittest
import sys
sys.path.append("..")
from tracker import FlightData

class TestTracker(unittest.TestCase):

    def test_control_points_all_same(self):
        fd = FlightData()
        t = range(10)
        input = [1]*10
        actual = fd.identify_control_points(t, input)

        expected = [ (t[0], input[0]) ]
        self.assertSequenceEqual(actual, expected) 

    def test_control_points_first_different(self):
        fd = FlightData()
        t = range(5)
        input = [1] + [2]*4

        actual = fd.identify_control_points(t, input)
        expected = [ (t[0], input[0]),
                     (t[1], input[1])
                    ]

        self.assertSequenceEqual(actual, expected) 

    def test_control_points_last_different(self):

        fd = FlightData()
        t = range(5)
        input = [1]*4 + [2]

        actual = fd.identify_control_points(t, input)
        expected = [ (t[0], input[0]),
                     (t[4], input[4])
                    ]

        self.assertSequenceEqual(actual, expected) 

    def test_control_points_varying(self):

        fd = FlightData()
        t = range(8)
        input = [ -1, 1, 4, 4, 7, 8, 8, 9]

        actual = fd.identify_control_points(t, input)
        expected = [ (t[0], input[0]),
                     (t[1], input[1]),
                     (t[2], input[2]),
                     (t[4], input[4]),
                     (t[5], input[5]),
                     (t[7], input[7]),
                    ]

        self.assertSequenceEqual(actual, expected) 


if __name__ == "__main__":
    unittest.main()
