#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time
import openravepy
import pickle

#### YOUR IMPORTS GO HERE ####
import numpy as np
from pr2 import PR2
from KalmanFilter import KalmanFilter
from tuning import fitting
from ParticleFilter import ParticleFilter
import matplotlib.pyplot as plt

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


def ConvertPathToTrajectory(env, robot,path=[]):
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
        self.actions = in_dict["action"].transpose()
        self.path = in_dict["groundtruth"].transpose()
        self.plan = in_dict['path']
        self.noisy_measurement = in_dict["sensor_data"].transpose()
        self.start = self.noisy_measurement[:, 0].reshape((-1,1))
        # self.start = self.path[:, 0].reshape((-1,1)) # self.noisy_measurement[:, 0].reshape((-1,1))
        self.filter = filter
        self.N = self.path.shape[1]

    def calculate_error(self, ground, estimated):
        state_errors = estimated[:,1:self.N] - ground[:,1:self.N]
        total_error=np.sum(np.linalg.norm(state_errors, axis=0))
        return total_error

    def simulate(self):
        # Set the robot to starting position
        curr_position = np.matrix(self.start)
        self.pr2.set_position(self.start.squeeze(),self.plan[0][2])
        Sigma = np.eye(2)

        R,Q = fitting(self.filename, self.pr2.A, self.pr2.B, self.pr2.C)

        actual_path = np.zeros((2, self.N))
        z0 = np.matrix(self.noisy_measurement[:,0]).T
        actual_path[:,0] = np.squeeze(np.array(np.linalg.inv(self.pr2.C) * z0))

        in_collision = []

        for i in range(1,self.N):
            z = np.matrix(self.noisy_measurement[:,i]).transpose() # self.pr2.gps_measurement() # 
            u = np.matrix(self.actions[:,i]).transpose()

            curr_position, Sigma = self.filter(curr_position, Sigma, z, u, np.matrix(self.pr2.A), np.matrix(self.pr2.B), np.matrix(self.pr2.C), np.matrix(Q), np.matrix(R))
            actual_path[:, i] = np.squeeze(np.array(curr_position))

            if i > 4:
                self.pr2.set_position(curr_position,self.plan[i][2])
                if self.env.CheckCollision(self.pr2.robot):
                    print "In collision ", i
                    in_collision.append(True)
                else:
                    in_collision.append(False)

            # self.pr2.set_position(self.path[:, i])

        print "Total Error: %f"%self.calculate_error(self.path, actual_path)

        rval = []
        for pt, planned in zip(actual_path.T,self.plan):
            rval.append(np.append(np.squeeze(pt),planned[2]))
        return self.path.transpose(), rval, in_collision


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

        #### YOUR CODE HERE ####
        PF = ParticleFilter(3000, [[-4.,4.],[-1.5,4.]],0.1,"multivariate_normal")
        sim = Simulator(env, robot, "data/env2_hwk3.pickle", PF.filter)
        
        # PF = ParticleFilter(10000,[[-4.,4.],[-1.5,4.]],0.1,'skewnorm', Q=[-10,0,0.5])
        # sim = Simulator(env, robot, "data/env2_skewnorm.pickle", PF.filter)

        # sim = Simulator(env, robot, "data/env2_hwk3.pickle", KalmanFilter)
        start = time.clock()
        ground_truth, actual_path, in_collision = sim.simulate()
        end = time.clock()

        print "Execution time: ", end-start

        ar = np.array(actual_path)
        plt.plot(ar[:,0],ar[:,1],label='estimation')
        gd = np.array(ground_truth)
        plt.plot(gd[:,0],gd[:,1],label='groundtruth')
        plt.legend()
        plt.show()

        if True in in_collision:
            print "Estimated Path is in Collision."

        # PLOTTING
        for pt in ground_truth:
            handles.append(env.plot3(points=(pt[0], pt[1], 0.3), pointsize=3.0, colors=(((0,0,0)))))
        for pt,collision in zip(actual_path, in_collision):
            if collision:
                handles.append(env.plot3(points=(pt[0], pt[1], 0.3), pointsize=5.0, colors=(((0,0,1)))))    
            else:
                handles.append(env.plot3(points=(pt[0], pt[1], 0.3), pointsize=5.0, colors=(((0,0,1)))))

        # Now that you have computed a path, convert it to an openrave trajectory 
        traj = ConvertPathToTrajectory(robot, actual_path)

    # Execute the trajectory on the robot.
    if traj != None:
        robot.GetController().SetPath(traj)


    waitrobot(robot)

    raw_input("Press enter to exit...")
