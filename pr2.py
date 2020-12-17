# Implements Functions needed from PR2 Robot
import numpy as np
from sensor import Sensor


class PR2:

    def __init__(self, robot):
        self.robot = robot
        self.A = np.eye(2)
        self.B = np.eye(2)
        self.C = np.eye(2)
        cov = [[ 1e-1, -8e-2 ],
               [ -8e-2, 1e-1]]
        self.sensor = Sensor(self.C, cov)

    def get_true_location(self):
        location = self.robot.GetTransform()[:, 3]
        return np.array([[location[0], location[1]]]).T

    def gps_measurement(self):
        return self.sensor.gen_noisy(np.matrix(self.get_true_location()))

    def set_position(self, coordinates):
        # coodinates must be a 1D array of size 2
        coordinates = list(coordinates)
        coordinates.append(0.05)
        try:
            self.robot.SetActiveDOFValues(np.array(coordinates))
        except TypeError:
            import pdb; pdb.set_trace() 
