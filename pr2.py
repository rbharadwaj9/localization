# Implements Functions needed from PR2 Robot
import numpy as np


class PR2:

    def __init__(self, robot):
        self.robot = robot
        self.A = np.eye(2)
        self.B = np.eye(2)
        self.C = np.eye(2)

    def get_true_location(self):
        location = self.robot.GetTransform()[:, 3]
        return np.array([[location[0], location[1]]]).T

    def set_position(self, coordinates):
        # coodinates must be a 1D array of size 2
        coordinates = list(coordinates)
        coordinates.append(0.05)
        try:
            self.robot.SetActiveDOFValues(np.array(coordinates))
        except TypeError:
            import pdb; pdb.set_trace() 
