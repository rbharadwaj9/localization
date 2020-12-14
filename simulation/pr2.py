# Implements Functions needed from PR2 Robot
import numpy as np


class PR2:

    def __init__(self, robot):
        self.robot = robot

    def get_true_location(self):
        location = self.robot.GetTransform()[:, 3]
        return np.array([location[0], location[1]])
