#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time
import openravepy
import numpy
#### YOUR IMPORTS GO HERE ####

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
        jointnames = ['torso_lift_joint','l_shoulder_lift_joint','l_elbow_flex_joint','l_wrist_flex_joint','r_shoulder_lift_joint','r_elbow_flex_joint','r_wrist_flex_joint']
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
        robot.SetActiveDOFValues([0.24,1.29023451,-2.32099996,-0.69800004,1.27843491,-2.32100002,-0.69799996]);        
        robot.GetController().SetDesired(robot.GetDOFValues());
    waitrobot(robot)

#set active DOF values from a numpy matrix
def SetActiveDOFValuesNPMatrix(robot,qmat):
    qo = [q.item(i) for i in range(0,qmat.shape[1])]
    robot.SetActiveDOFValues(qo)


#returns the end effector transform in the world frame
def GetEETransform(robot,activedofvalues=None):
    if activedofvalues != None:
        robot.SetActiveDOFValues(activedofvalues);
    manip = robot.GetActiveManipulator()
    return manip.GetEndEffectorTransform()

#returns the joint axis in the world frame
def GetJointAxis(robot,jointname):
    return robot.GetJoint(jointname).GetAxis(0)

#returns the joint position in the world frame
def GetJointPosition(robot,jointname):
    return robot.GetJoint(jointname).GetAnchor()

def GetTranslationJacobian(robot,jointnames):
    J = numpy.zeros((3, robot.GetActiveDOF()))
    ### YOUR CODE HERE ###
    initial_config = robot.GetActiveDOFValues()
    temp = GetEETransform(robot, initial_config)
    initial_end_posi = array([temp[0][3], temp[1][3], temp[2][3]])
    for i in range(robot.GetActiveDOF()):
        v1 = GetJointAxis(robot, jointnames[i])
        joint_posi = GetJointPosition(robot, jointnames[i])
        p1 = array(initial_end_posi)-array(joint_posi)
        V = numpy.cross(v1, p1)
        J[0][i] = V[0]
        J[1][i] = V[1]
        J[2][i] = V[2]
    ### YOUR CODE HERE ###
    return J

def GetJpinv(J):
    ### YOUR CODE HERE ###
    Jpinv = []
    J_temp = matrix(J)
    num_row = shape(J_temp)[0]
    if abs(numpy.linalg.det(J_temp.T*J_temp)) > 0.01:
        Jpinv = J_temp.T*numpy.linalg.inv(J_temp*J_temp.T)
    else:
        #print 'Not full rank'
        Jpinv = J_temp.T*numpy.linalg.inv(J_temp*J_temp.T+0.1*numpy.identity(num_row))
    ### YOUR CODE HERE ###
    return Jpinv


if __name__ == "__main__":

    env = Environment()
    env.SetViewer('qtcoin')
    # env.SetDebugLevel(1) #comment this in to turn off openrave warnings
    collisionChecker = RaveCreateCollisionChecker(env,'ode')
    env.SetCollisionChecker(collisionChecker)

    env.Reset()        
    # load a scene from environment XML file
    env.Load('pr2only.env.xml')
    time.sleep(0.1)

    # 1) get the 1st robot that is inside the loaded scene
    # 2) assign it to the variable named 'robot'
    robot = env.GetRobots()[0]

    # tuck in the PR2's arms and raise torso
    tuckarms(env,robot);
  
    #set start config
    robot.SetActiveManipulator('leftarm')
    jointnames =['l_shoulder_pan_joint','l_shoulder_lift_joint','l_upper_arm_roll_joint','l_elbow_flex_joint','l_forearm_roll_joint','l_wrist_flex_joint','l_wrist_roll_joint']
    robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])      


    targets = [[-0.15070158,  0.47726995,  1.56714123],
           [-0.36535318,  0.11249, 1.08326675],
           [-0.56491217,  0.011443, 1.2922572 ],
           [-1.07012697,  0.81909669,  0.47344636],
           [-1.11050811,  0.97000718,  1.31087581]]
    doflimits = robot.GetActiveDOFLimits() #make sure q doesn't go past these limits
    q = numpy.zeros((1,robot.GetActiveDOF())) #start at this configuration
    with env:
        start = time.clock()
        handles = [] #graphics handles for plotting
        SetActiveDOFValuesNPMatrix(robot,q)

        ### YOUR CODE HERE ###
        target = targets[0] ###pick your target here
        doflimits[0][4] = -pi/2
        doflimits[0][6] = -pi/2
        doflimits[1][4] = pi/2
        doflimits[1][6] = pi/2
        #draw the target point in blue
        handles.append(env.plot3(points=array(target), pointsize=15.0, colors=array((0,0,1)) )) 
        q0 = robot.GetActiveDOFValues()
        q_current = array(q0)
        thredhold = 0.01
        alpha = 1
        while True:
            temp_x = GetEETransform(robot, q_current.tolist())
            x_current = array([temp_x[0][3], temp_x[1][3], temp_x[2][3]])
            d_x = target - x_current
            if numpy.linalg.norm(d_x) < thredhold:
                break
            J_current = GetTranslationJacobian(robot,jointnames)
            J_current_inv = GetJpinv(J_current)
            q_prime = J_current_inv * matrix(d_x).T
            if numpy.linalg.norm(q_prime) > alpha:
                q_prime = alpha * q_prime/numpy.linalg.norm(q_prime)
            q_current = q_current + numpy.squeeze(numpy.asarray(q_prime))
            for i in range(7):
                if q_current[i] < doflimits[0][i]:
                    q_current[i] = doflimits[0][i]
                elif q_current[i] > doflimits[1][i]:
                    q_current[i] = doflimits[1][i]
            robot.SetActiveDOFValues(q_current.tolist())
        """
        print doflimits[0]
        print doflimits[1]
        """
        print 'Configuration:', q_current

        ### YOUR CODE HERE ###

    robot.GetController().SetDesired(robot.GetDOFValues())
    waitrobot(robot)

    raw_input("Press enter to exit...")
    env.Destroy()
