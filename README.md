# EECS 498 Final Project - Localization
EECS 498 - Fall 2020 <br>
Prof. Dmitry Berenson <br>
GSI: Anthony Liang <br>
<br>
Bo-Ray Yeh (boray@umich.edu) <br>
Rajiv Bharadwaj (rajivbh@umich.edu)<br>
<br>
## Running the Demonstration Program `demo.py`
This performs a demonstration of the simulator by taking the data from `data/env2_circle_back.pickle` and shows the OpenRAVE and Matplotlib output of the estimations for each algorithm.

**Terminal Output:** Outputs error and time taken to run each simulation. Also mentions whether the estimated path might be in collision. Additionally, there are breakpoints set to play the trajectory estimated from each simulation which require keyboard input from the user.    

**OpenRAVE Viewer Output:** The ground truth is plotted in black. Estimations from the Particle Filter and Kalman Filter are points in blue and green respectively. Points in collision for the Particle and Kalman Filter are in red and purple respectively.   
This is also where the estimated path can be played.  

**Matplotlib Output:** There are three figures plotted. The first figure shows the same output from the OpenRAVE environment. However, it’s shown in 2 dimensions to make it visually comparable. The second figure shows just the Kalman Filter vs Ground Estimation. The third figure shows the Particle Filter vs Ground Estimation.      The color code is the same as the OpenRAVE one.    

## Guide to the repository

#### Simulation Class
Simulation class is the main function that we will be using for OpenRAVE stuff. <br>
It is in `simulator.py`
1. All the supporting libraries are in the top level directory of the repository to simplify things when we submit the project.
2. `KalmanFilter.py` contains `KalmanFilter(args)` 
3. `tuning.py` takes the file concerned in `simulator.py` and returns the noise distributions
4. `pr2.py` defines a PR-2 class that abstracts a lot of the robot's actions taken with OpenRAVE.
5. `sensor.py` defines the sensor class for the PR-2 robot.

#### Kalman Filter
Exists in `KalmanFilter.py` <br>
It defines `KalmanFilter` function to pass as an argument to the simulator class.

#### Particle Filter
Exists in `ParticleFilter.py`<br>
It defines `ParticleFilter` class to handle the data need to store and method for each action and sensing.<br>
There is a demo main function inside, so just call it and it would plot path estimation and scatter plot for weight calculation

#### Path Generation
Exists in `path_gen/` <br>
Generates an A\* path based on the waypoints given in `goalconfig` into a pickle file.
The path is saved as `path` <br>

#### Ground Truth Generation
Exists in `action_groundtruth_gen/`
Takes in the path generated above and adds motion noise and action data to it. <br>
Ground Truth is saved as `groundtruth` <br>
Action is saved as `actions`

#### Sensor Measurement Generation
Exists in `sensor_gen/` <br>
Takes in the ground truth file above and adds in sensor measurement as `sensor_data` in the pickle file.

## Data
All the data collected exists in `data/` <br>
The files are pickle files with `env<num>_<description>.pickle` The env number is the xml file that we used to generate the A\* path in

## For Developers

#### Kalman Filtering - Test
Exists in `kf/` <br>
This is where the robot's data can be tested and plotted with matplotlib. Essentially the same as simulator but without OpenRAVE

#### Particle Filtering - Test
Exists in `pf/` <br>
This is where the robot's data can be tested and plotted with matplotlib. Essentially the same as simulator but without OpenRAVE

## Requirements
Requires Python 2 with OpenRAVE, Matplotlib, Numpy, Scipy
