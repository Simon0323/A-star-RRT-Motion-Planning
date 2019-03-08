#!/usr/bin/env python
# -*- coding: utf-8 -*-raw_input("Press enter to exit...")
import time
import openravepy

#### YOUR IMPORTS GO HERE ####
from demo_fun import Kinodynamic_RRT
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
    planningutils.RetimeAffineTrajectory(traj,maxvelocities=2*ones(3),maxaccelerations=5*ones(3))
    return traj


if __name__ == "__main__":

    env = Environment()
    env.SetViewer('qtcoin')
    collisionChecker = RaveCreateCollisionChecker(env,'ode')
    env.SetCollisionChecker(collisionChecker)


    env.Reset()
    # load a scene from ProjectRoom environment XML file
    env.Load('pr2env_demo.xml')
    time.sleep(0.1)

    # 1) get the 1st robot that is inside the loaded scene
    # 2) assign it to the variable named 'robot'
    robot = env.GetRobots()[0]

    # tuck in the PR2's arms for driving
    tuckarms(env,robot);


    with env:
        # the active DOF are translation in X and Y and rotation about the Z axis of the base of the robot.
        robot.SetActiveDOFs([], DOFAffine.X|DOFAffine.Y|DOFAffine.RotationAxis,[0,0,1])

        goalconfig = [-1,-1.3,-pi/2]
        start = time.clock()
        #### YOUR CODE HERE ####
        #### Implement your algorithm to compute a path for the robot's base starting from the current configuration of the robot and ending at goalconfig. The robot's base DOF have already been set as active. It may be easier to implement this as a function in a separate file and call it here.
        #print 'Robots Initial Positipresent_state = temp_stateon', robot.GetTransform()
        _path = []
        smooth_ite = 150
        Initialposi = robot.GetTransform()
        initialconfig = array([Initialposi[0][3], Initialposi[1][3], arccos(Initialposi[0][0])])
        handles = []
        initial_state = matrix([Initialposi[0][3], Initialposi[1][3], arccos(Initialposi[0][0]), 0, 0, 0]).T
        goal_state = matrix([1, -3.6, pi/4, 0, 0, 0]).T
        state_limit = matrix([[-10, -5, -pi, -8, -6, -2], [10, 5, pi, 8, 6, 2]]).T
        action_limit = matrix([[-8, -5, -3], [8, 5, 3]]).T
        _path = Kinodynamic_RRT(initial_state, goal_state, state_limit, action_limit, env, robot, handles, smooth_ite)

        #### Draw the X and Y components of the configurations explored by your algorithm

        path = _path #put your final path in this variable
        #### END OF YOUR CODE ###
        end = time.clock()
        print "Time: ", end - start

        # Now that you have computed a path, convert it to an openrave trajectory
        traj = ConvertPathToTrajectory(robot, path)

    # Execute the trajectory on the robot.
    if traj != None:
        robot.GetController().SetPath(traj)


    waitrobot(robot)

    raw_input("Press enter to exit...")

