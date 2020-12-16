#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time
import openravepy
import pickle

#### YOUR IMPORTS GO HERE ####
import numpy as np
from pr2 import PR2
from KalmanFilter import KalmanFilter

#### END OF YOUR IMPORTS ####

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)

def tuckarms(env,robot):
    with env:
        jointnames = ['l_shoulder_lift_joint','l_elbow_flex_joint','l_wrist_flex_joint','r_shoulder_lift_joint','r_elbow_flex_joint','r_wrist_flex_joint']
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
        robot.SetActiveDOFValues([1.29023451,-2.32099996,-0.69800004,1.27843491,-2.32100002,-0.69799996]);
        robot.GetController().SetDesired(robot.GetDOFValues());
    waitrobot(robot)


def ConvertPathToTrajectory(robot,path=[]):
#Path should be of the form path = [q_1, q_2, q_3,...], where q_i = [x_i, y_i, theta_i]

    if not path:
        return None
    # Initialize trajectory
    traj = RaveCreateTrajectory(env,'')    
    traj.Init(robot.GetActiveConfigurationSpecification())
    for i in range(0,len(path)):
        traj.Insert(i,numpy.array(path[i]))
    # Move Robot Through Trajectory
    planningutils.RetimeAffineTrajectory(traj,maxvelocities=ones(3),maxaccelerations=5*ones(3))
    return traj

class Simulator:

    def __init__(self, env, robot, filename, filter):
        self.env = env
        self.pr2 = PR2(robot)
        self.filename = filename

        with open(filename, "rb") as f:
            in_dict = pickle.load(f)
            self.actions = in_dict["action"]
            self.path = in_dict["groundtruth"]
        self.start = self.path[0, :]
        self.filter = filter
        self.N = self.path.shape(0)

    def simulate(self):
        # Set the robot to starting position
        curr_position = self.start
        self.robot.set_position(self.start.squeeze())
        Sigma = np.eye(2)

        R,Q = fitting(self.filename)

        actual_path = np.zeros((2, self.N))
        for action in self.actions:
            z = self.robot.get_true_location()
            u = action

            next_position, Sigma = self.filter(curr_position, Sigma, z, u, self.robot.A, self.robot.B, self.robot.C, Q, R)
            self.robot.set_position(next_position)
            actual_path[:, i] = np.squeeze(next_position)

            if env.CheckCollision(robot):
                print "In collision"
                break
        return self.path, actual_path


if __name__ == "__main__":

    env = Environment()
    env.SetViewer('qtcoin')
    collisionChecker = RaveCreateCollisionChecker(env,'ode')
    env.SetCollisionChecker(collisionChecker)


    env.Reset()
    # load a scene from ProjectRoom environment XML file
    env.Load('data/pr2test2.env.xml')
    time.sleep(0.1)

    # 1) get the 1st robot that is inside the loaded scene
    # 2) assign it to the variable named 'robot'
    robot = env.GetRobots()[0]

    # tuck in the PR2's arms for driving
    tuckarms(env,robot);
    handles = []

    with env:
        # the active DOF are translation in X and Y and rotation about the Z axis of the base of the robot.
        robot.SetActiveDOFs([],DOFAffine.X|DOFAffine.Y|DOFAffine.RotationAxis,[0,0,1])

        goalconfig = [2.6,-1.3,-pi/2]

        start = time.clock()
        #### YOUR CODE HERE ####
        sim = Simulator(env, robot, file, KalmanFilter)
        ground_truth, actual_path = sim.simulate()

        # PLOTTING
        for pt in ground_truth:
            handles.append(env.plot3(points=(pt.x, pt.y, 0.3), pointsize=3.0, colors=(((0,0,1)))))
        for pt in actual_path:
            handles.append(env.plot3(points=(pt[0], pt[1], 0.3), pointsize=5.0, colors=(((0,0,0)))))


        # Now that you have computed a path, convert it to an openrave trajectory 
        traj = ConvertPathToTrajectory(robot, path)

    # Execute the trajectory on the robot.
    if traj != None:
        robot.GetController().SetPath(traj)


    waitrobot(robot)

    raw_input("Press enter to exit...")
