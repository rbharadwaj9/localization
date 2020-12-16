#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time
import openravepy
import pickle

#### YOUR IMPORTS GO HERE ####
import numpy as np
from pr2 import PR2

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

    def __init__(self, env, robot, path, actions, filter):
        self.env = env
        self.pr2 = PR2(robot)
        self.path = path
        self.start = path[0, :]
        self.actions = actions
        self.filter = filter
        self.N = self.path.shape(0)

    def simulate(self):
        # Set the robot to starting position
        curr_position = self.start
        self.robot.set_position(self.start.squeeze())
        Sigma = np.eye(2)

        from kfmodel import A, B, C
        R,Q = fitting()

        for action in self.actions:
            z = self.robot.get_true_location()
            u = action

            next_position, Sigma = self.filter(curr_position, Sigma, z, u, A, B, C, Q, R)

            estimated_states[:, i] = np.squeeze(next_position)





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
        sim = Simulator(env, robot)
        sim.pr2.get_true_location()

        #### Implement your algorithm to compute a path for the robot's base starting from the current configuration of the robot and ending at goalconfig. 
        #### The robot's base DOF have already been set as active. It may be easier to implement this as a function in a separate file and call it here.

        # Now that you have computed a path, convert it to an openrave trajectory 
        traj = ConvertPathToTrajectory(robot, path)

    # Execute the trajectory on the robot.
    if traj != None:
        robot.GetController().SetPath(traj)


    waitrobot(robot)

    raw_input("Press enter to exit...")
