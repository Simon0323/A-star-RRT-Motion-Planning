#!/usr/bin/env python
# -*- coding: utf-8 -*-
import openravepy
import numpy
import random
from copy import deepcopy

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

class Node:
    def __init__(self, _input, u, id_in, parentid_in):
        self.state = _input
        self.control = u
        self.id = id_in
        self.parentid = parentid_in

    def printme(self):
        print self.state[0], self.state[1], self.state[2], self.state[3], self.state[4], self.state[5],self.control, self.id, self.parentid

def Random_state(bias_ratio, limit, goal_state):
    alpha = 1
    if random.random() <= bias_ratio:
        if random.random() <= 0.4:
            randconfig = goal_state
        else:
            randconfig = goal_state
    else:
        """
        randconfig = matrix([random.uniform(limit[0, 0], limit[0, 1]), random.uniform(limit[1, 0], limit[1, 1]),
                            pi/4, alpha*random.uniform(limit[3, 0], limit[3, 1]),
                            alpha*random.uniform(limit[4, 0], limit[4, 1]), 0]).T
        """
        randconfig = matrix([random.uniform(limit[0, 0], limit[0, 1]), random.uniform(limit[1, 0], limit[1, 1]),
                            random.uniform(limit[2, 0], limit[2, 1]), alpha*random.uniform(limit[3, 0], limit[3, 1]),
                            alpha*random.uniform(limit[4, 0], limit[4, 1]), alpha*random.uniform(limit[5, 0], limit[5, 1])]).T
    return randconfig

def Getclosest_Node(all_node_state, rand_state, weight):
    dis = numpy.linalg.norm(numpy.diag(weight)*(all_node_state - rand_state), axis=0)
    index_min = numpy.argmin(dis)
    return index_min

def Iscollision(env, robot, newconfig):
    transform_matrix = array([[cos(newconfig[2]), -sin(newconfig[2]), 0, newconfig[0]],
                              [sin(newconfig[2]), cos(newconfig[2]), 0, newconfig[1]],
                              [0, 0, 1, 0.05],
                              [0, 0, 0, 1]])
    robot.SetTransform(transform_matrix)
    if env.CheckCollision(robot) is False:
        return 0
    else:
        return 1

def Disfunction(present_state, goal_state, w):
    return (present_state - goal_state).T * numpy.diag(w) * (present_state - goal_state)

def Plot(env, handles, posi, color):
    if color == 0:
        color = (1, 0, 1)
    elif color == 1:
        color = (1, 0, 0)
    else:
        color = (0, 0, 1)
    handles.append(env.plot3(points=array(posi),
                                   pointsize = 10,
                                   colors=array((color))))
def PlotPath(path, env, handles):
    for i in range(len(path)):
        handles.append(env.plot3(points=array((path[i][0], path[i][1], 0.05)),
                                   pointsize = 6,
                                   colors=array(((1, 0, 0)))))

def CreatPath(allnode, goalid, env, handles):
    path = []
    path_fullstate = []
    id_present = goalid
    while True:
        state_p = [allnode[id_present].state[0, 0], allnode[id_present].state[1, 0], allnode[id_present].state[2, 0]]
        state_full = allnode[id_present].state
        path.append(state_p)
        path_fullstate.append(state_full)
        handles.append(env.plot3(points=array((state_p[0], state_p[1], 0.05)),
                                   pointsize = 6,
                                   colors=array(((0, 0, 0)))))
        if id_present == 0:
            break
        else:
            id_present = allnode[id_present].parentid
    path.reverse()
    path_fullstate.reverse()
    print 'Path generated'
    return path, path_fullstate

def ConvertStatetoPath(state_smoothed):
    path_smoothed = []
    for i in range(len(state_smoothed)):
        path_smoothed.append(state_smoothed[i][0:3])
    return path_smoothed

def PathSmoothing(path_unsmooth, ite_times, control, A, B, A_star, step_num, u, weight_dis, thred_ter, env, robot, handles):
    path_smooth = deepcopy(path_unsmooth)
    control_inv = control.T*numpy.linalg.inv(control*control.T)
    flag_colli_free = 1
    #######
    #to comput a optimal input in each period, to avoid the unknown of the need step
    #######
    """
    for i in range(ite_times):
        temp1 = random.randint(0, len(path_smooth))
        temp2 = random.randint(0, len(path_smooth))
        index1 = min(temp1, temp2)
        index2 = max(temp1, temp2)
        if abs(index2-index1) < 2:
            continue
        start_state = deepcopy(path_smooth[index1])
        target_state = deepcopy(path_smooth[index2])
        present_state = deepcopy(start_state)
        new_state_list = []
        while True:
            new_state = A * present_state + B * u
            index_u = Getclosest_Node(new_state, target_state, weight_dis)
            temp_state = deepcopy(new_state[:, index_u])
            if Iscollision(env, robot, temp_state) == 1:
                flag_colli_free = 0
                break
            present_state = deepcopy(temp_state)
            new_state_list.append(deepcopy(present_state))
            handles.append(env.plot3(points=array((temp_state[0], temp_state[1], 0.05)),
                                   pointsize = 20,
                                   colors=array(((0, 0.5, 0.5)))))
            if Disfunction(present_state, target_state, weight_dis) < thred_ter:
                break
        if flag_colli_free == 1:
            del path_smooth[index1+1:index2]
            path_smooth[index1+1:index1+1] = new_state_list
    """

    #######
    #smoothing use the method of linear system, to direct comput a input make the state from start to the target state
    #######

    for i in range(ite_times):
        temp1 = random.randint(0, len(path_smooth))
        temp2 = random.randint(0, len(path_smooth))
        index1 = min(temp1, temp2)
        index2 = max(temp1, temp2)
        if abs(index2-index1) < 16:
            continue
        start_state = deepcopy(path_smooth[index1])
        target_state = deepcopy(path_smooth[index2])
        u = control_inv*(target_state - A_star*start_state)
        u = numpy.reshape(u, (step_num, 3)).T
        temp_state_list = []
        temp_state = start_state
        flag_colli_free = 1
        for j in range(step_num):
            u_present = u[:, step_num-1-j]
            temp_state = A * temp_state + B * u_present
            temp_state_list.append(deepcopy(temp_state))
            """
            handles.append(env.plot3(points=array((temp_state[0], temp_state[1], 0.05)),
                                   pointsize = 10,
                                   colors=array(((0, 0.5, 0.5)))))

            """
            if Iscollision(env, robot, temp_state) == 1:
                flag_colli_free = 0
                break
        if flag_colli_free == 1:
            del path_smooth[index1+1:index2+1]
            path_smooth[index1+1:index1+1] = temp_state_list

    return path_smooth


def Kinodynamic_RRT(initial_state, goal_state, state_limit, action_limit, env, robot, handles, smooth_ite):
    allnode = []
    num_ite = 0
    allnode_state = initial_state
    id_present = 0
    flag_terminate = 0
    thred_fina_ter = 0.1
    thred_ter = 0.1
    #some constant matrix
    weight_dis = array([1.4, 1.4, 1, 0.8, 0.8, 0.8])
    weight_dis = weight_dis / numpy.linalg.norm(weight_dis)
    weight_ter = weight_dis
    #weight_ter = array([1, 1, 0.35, 0.3, 0.3, 0.3])
    d_t = 0.025
    A = matrix([[1, 0, 0, d_t, 0, 0], [0, 1, 0, 0, d_t, 0], [0, 0, 1, 0, 0, d_t], [0, 0, 0, 1, 0, 0], [0, 0, 0, 0, 1, 0], [0, 0, 0, 0, 0, 1]])
    B = matrix([[0, 0, 0], [0, 0, 0], [0, 0, 0], [d_t, 0, 0], [0, d_t, 0], [0, 0, d_t]])
    step_num = 40
    control = numpy.hstack((B, A*B, A**2*B, A**3*B, A**4*B, A**5*B, A**6*B, A**7*B, A**8*B, A**9*B,
                            A**10*B, A**11*B, A**12*B, A**13*B, A**14*B, A**15*B, A**16*B, A**17*B, A**18*B, A**19*B,
                            A**20*B, A**21*B, A**22*B, A**23*B, A**24*B, A**25*B, A**26*B, A**27*B, A**28*B, A**29*B,
                            A**30*B, A**31*B, A**32*B, A**33*B, A**34*B, A**35*B, A**36*B, A**37*B, A**38*B, A**39*B))
    A_star = A**step_num
    """
    u_x = array([-5, -2.5, 0, 2.5, 5])
    u_y = array([-3, -1.5, 0, 1.5, 3])
    u_r = array([-1, -0.5, 0, 0.5, 1])
    """
    """
    u_x = array([-5, 0,  5])
    u_y = array([-3, 0,  3])
    u_r = array([-1, 0,  1])
    """

    u_x = array([-5, -4, -3, -2, -1, -0.5, 0, 0.5, 1, 2, 3, 4, 5])
    u_y = array([-3, -2, -1, -0.5, 0, 0.5, 1, 2, 3])
    u_r = array([-1, -0.8, -0.6, -0.4, -0.2, 0, 0.2, 0.4, 0.6, 0.8, 1])

    u = numpy.zeros((3, len(u_x)*len(u_y)*len(u_r)))
    temp = 0
    for i in range(len(u_x)):
        for j in range(len(u_y)):
            for k in range(len(u_r)):
                u[:, temp] = [u_x[i], u_y[j], u_r[k]]
                temp += 1
    u_all = u
    for i in range(step_num-1):
        u_all = numpy.vstack((u_all, u))
    #plot
    handles.append(env.plot3(points=array((goal_state[0], goal_state[1], 0.05)),
                                           pointsize = 20,
                                           colors=array(((1, 0, 1)))))
    while True:
        target_p_state = Random_state(0.1, state_limit, goal_state)
        id_par = Getclosest_Node(allnode_state, target_p_state, weight_dis)
        present_state = deepcopy(allnode_state[:, id_par])
        index_u = Getclosest_Node(A_star * present_state + control * u_all, target_p_state, weight_dis)
        u_present = numpy.reshape(deepcopy(u[:, index_u]),(3,1))
        """
        handles.append(env.plot3(points=array((target_p_state[0], target_p_state[1], 0.05)),
                                           pointsize = 10,
                                           colors=array(((1, 0, 0)))))
        """
        for i in range(step_num):
            temp_state = A * present_state + B * u_present
            if Iscollision(env, robot, temp_state) == 1:
                break
            allnode.append(deepcopy(Node(present_state, u_present, id_present, id_par)))

            handles.append(env.plot3(points=array((temp_state[0], temp_state[1], 0.05)),
                                           pointsize = 2,
                                           colors=array(((0, 0, 1)))))

            id_par = id_present
            id_present += 1
            if id_present != 1:
                allnode_state = numpy.hstack((allnode_state, deepcopy(present_state)))
            if Disfunction(present_state, goal_state, weight_ter) < thred_fina_ter:
                goalid = id_par
                flag_terminate = 1
                #allnode[goalid].printme()
                break
            if Disfunction(present_state, target_p_state, weight_ter) < thred_ter:
                break
            present_state = deepcopy(temp_state)
            num_ite += 1
            if num_ite % 4000 == 0:
                print 'Number of Nodes = ', num_ite
        if flag_terminate == 1:
            break
    path_unsmooth, path_fullstate = CreatPath(allnode, goalid, env, handles)
    state_smooth = PathSmoothing(path_fullstate, smooth_ite, control, A, B, A_star, step_num, u, weight_dis, 0.1, env, robot, handles)
    path_smooth = ConvertStatetoPath(state_smooth)
    PlotPath(path_smooth, env, handles)
    #raw_input("Press enter to exit...")
    return path_smooth




















