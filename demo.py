#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time
import openravepy
import pickle

# EECS 498 Project Demo File
# Rajiv Bharadwaj (rajivbh), Bo-Ray Yeh (boray)


import numpy as np
from simulator import Simulator, tuckarms, ConvertPathToTrajectory, waitrobot
from ParticleFilter import ParticleFilter
from KalmanFilter import KalmanFilter
import matplotlib.pyplot as plt

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *


def main():
    env = Environment()

    env.SetViewer('qtcoin')
    collisionChecker = RaveCreateCollisionChecker(env, 'ode')
    env.SetCollisionChecker(collisionChecker)

    env.Reset()
    env.Load('path_gen/pr2test2.env.xml')
    time.sleep(0.1)

    robot0 = env.GetRobots()[0]
    tuckarms(env, robot0)

    handles = []
    kf_traj = None
    pf_traj = None

    filename = "data/env2_circle_back.pickle"
    with env:
        # the active DOF are translation in X and Y and rotation about the Z axis of the base of the robot.
        robot0.SetActiveDOFs([],DOFAffine.X|DOFAffine.Y|DOFAffine.RotationAxis,[0,0,1])

        # ******* Printout expected time to run ****** 
        print "Estimated time to run: 10~12 minutes"
        print "Ground truth: black points"
        print "Estimation of Particle Filter: blue points/ red if in-collision"
        print "Estimation of Kalman Filter: green points/ purple if in-collision"

        # ******* PARTICLE FILTER *******
        PF = ParticleFilter(2000, [[-4.0, 4.0], [-1.5, 4.0]], 0.1, 'multivariate_normal')
        pf_sim = Simulator(env, robot0, filename, PF.filter)
        print "****** Particle Filter ******"

        start = time.clock()
        pf_ground_truth, pf_actual_path, pf_in_collision = pf_sim.simulate()
        end = time.clock()


        print "PF Test for ", filename
        print "Execution Time: ", end-start
        if True in pf_in_collision:
            print "Estimated Path is in Collision"

        # Plotting the Data

        for pt in pf_ground_truth:
            handles.append(env.plot3(points=(pt[0], pt[1], 0.3), pointsize=3.0, colors=(((0,0,0)))))

        for pt,collision in zip(pf_actual_path, pf_in_collision):
            if collision:
                handles.append(env.plot3(points=(pt[0], pt[1], 0.3), pointsize=5.0, colors=(((1,0,0)))))    
            else:
                handles.append(env.plot3(points=(pt[0], pt[1], 0.3), pointsize=5.0, colors=(((0,0,1)))))

        pf_traj = ConvertPathToTrajectory(env, robot0, pf_actual_path)


        # ******* KALMAN FILTER *******
        kf_sim = Simulator(env, robot0, filename, KalmanFilter)
        print "****** Kalman Filter ******"

        start = time.clock()
        kf_ground_truth, kf_actual_path, kf_in_collision = kf_sim.simulate()
        end = time.clock()

        print "KF Test for ", filename
        print "Execution Time: ", end-start
        if True in kf_in_collision:
            print "Estimated Path is in Collision"


        # Plotting the Data

        for pt,collision in zip(kf_actual_path, kf_in_collision):
            if collision:
                handles.append(env.plot3(points=(pt[0], pt[1], 0.6), pointsize=5.0, colors=(((90.0/255.0, 0/255.0, 160.0/255.0)))))    
            else:
                handles.append(env.plot3(points=(pt[0], pt[1], 0.6), pointsize=5.0, colors=(((0,1,0)))))

        kf_traj = ConvertPathToTrajectory(env, robot0, kf_actual_path)

        # Plotting by matplotlib
        plt.ion()
        ax = plt.subplot(111)
        
        gt_points = np.array(pf_ground_truth)
        pf_points = np.array(pf_actual_path)
        kf_points = np.array(kf_actual_path)

        ax.plot(gt_points[:,0],gt_points[:,1],c='k',label='Ground truth')
        ax.plot(pf_points[:,0],pf_points[:,1],c='b',label='Estimation PF')
        ax.plot(kf_points[:,0],kf_points[:,1],c='g',label='Estimation KF')
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_title('Estimations of KF and PF with ground truth')
        ax.legend()
        plt.pause(0.01)


    raw_input("Press to play particle path")
    if pf_traj:
        robot0.GetController().SetPath(pf_traj)

    waitrobot(robot0)

    raw_input("Press to play kalman path")

    if kf_traj:
        robot0.GetController().SetPath(kf_traj)

    waitrobot(robot0)

    raw_input("Press enter to exit...")


if __name__ == "__main__":
    main()
