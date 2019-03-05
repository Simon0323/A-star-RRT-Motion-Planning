#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time
import openravepy
from openravepy import *
from numpy import *

#### YOUR IMPORTS GO HERE ####
from Queue import PriorityQueue
from priorityqueue_example import Node

def Isexist(state_present, closedlist):
    length = len(closedlist)
    for i in range(length):
        if (state_present[0] == closedlist[i].x) and (state_present[1] == closedlist[i].y) and (abs(state_present[2]-closedlist[i].theta) < 0.1):
            return 1
    return 0

def Iscollision(env, robot, newconfig):
    transform_matrix= array ([[cos(newconfig[2]), -sin(newconfig[2]), 0, newconfig[0]],
                              [sin(newconfig[2]), cos(newconfig[2]), 0, newconfig[1]],
                              [0, 0, 1, 0.05],
                              [0, 0, 0, 1]])
    robot.SetTransform(transform_matrix)
    if env.CheckCollision(robot) is False:
        return 0
    else:
        return 1


def Totalcost(pathcost_par, newconfig, lastconfig, goalconfig):
    c_present = sqrt((newconfig[0] - lastconfig[0])**2 + (newconfig[1] - lastconfig[1])**2 + (min(abs(newconfig[2]-lastconfig[2]), 2*pi-abs(newconfig[2]-lastconfig[2])))**2)
    h_present = sqrt((newconfig[0] - goalconfig[0])**2 + (newconfig[1] - goalconfig[1])**2 + (min(abs(newconfig[2]-goalconfig[2]), 2*pi-abs(newconfig[2]-goalconfig[2])))**2)
    cost_presnt = pathcost_par +c_present
    f_present = cost_presnt + h_present
    return f_present, cost_presnt


def IsTemination(presentconfig, goalconfig):
    if (abs(presentconfig[0]-goalconfig[0]) < 0.1) and (abs(presentconfig[1]-goalconfig[1]) < 0.1) and (abs(presentconfig[2]-goalconfig[2]) < pi/40):
        return 1
    return 0

def Creatpath(closedlist, goalid, env, handles):
    path = []
    idpresent=goalid
    while True:
        state_p = [closedlist[idpresent].x, closedlist[idpresent].y, closedlist[idpresent].theta]
        #print state_p[0], state_p[1], state_p[2], idpresent, closedlist[idpresent].parentid, closedlist[idpresent].pathcost
        path.append(state_p)
        handles.append(env.plot3(points=array((state_p[0], state_p[1], 0.05)),
                                   pointsize = 10,
                                   colors=array(((0, 0, 0)))))
        if idpresent == 0:
            break
        else:
            idpresent = closedlist[idpresent].parentid
    path.reverse()
    print 'Toal cost = ', closedlist[goalid].pathcost
    return path


def Astar(step_xy, step_y, step_theta, initialconfig, goalconfig, env, robot, handles, number_connect):
    q = PriorityQueue()
    node_initial = Node(initialconfig[0], initialconfig[1], initialconfig[2], 0, 0, 0)
    f_initial, pathcost_ini = Totalcost(0, initialconfig, initialconfig, goalconfig)
    q.put((f_initial, node_initial))
    closedlist = []
    allnode = []
    allnode.append(node_initial)
    id_newstate = 0
    goalid = 0
    flag = 0
    while q is not empty:
        node_present = q.get()
        x_par = node_present[1].x
        y_par = node_present[1].y
        theta_par = node_present[1].theta
        id_par = node_present[1].id
        pathcost_par = node_present[1].pathcost
        presentconfig = array((x_par, y_par, theta_par))
        if Isexist(presentconfig, closedlist) == 0:
            closedlist.append(node_present[1])
            #print presentconfig[0], presentconfig[1], presentconfig[2], '\n'
            if IsTemination(presentconfig, goalconfig) == 1:
                goalid = id_par
                print 'Goal state', presentconfig[0],presentconfig[1],presentconfig[2], id_par
                flag = 1
                break

            step_theta_p1 = (theta_par + step_theta)
            step_theta_p2 = (theta_par - step_theta)
            if step_theta_p1 > pi:
                step_theta_p1 = step_theta_p1 - 2*pi
            if step_theta_p2 < -pi:
                step_theta_p2 = step_theta_p2 + 2*pi

            if number_connect == 4:
                state_temp = array(((x_par + step_xy, y_par, theta_par),
                                    (x_par - step_xy, y_par, theta_par),
                                    (x_par, y_par + step_y, theta_par),
                                    (x_par, y_par - step_y, theta_par),
                                    (x_par, y_par, step_theta_p1),
                                    (x_par, y_par, step_theta_p2),))
            else:
                state_temp = array(((x_par + step_xy, y_par, theta_par),
                                    (x_par - step_xy, y_par, theta_par),
                                    (x_par, y_par + step_y, theta_par),
                                    (x_par, y_par - step_y, theta_par),
                                    (x_par + step_xy, y_par + step_y, theta_par),
                                    (x_par + step_xy, y_par - step_y, theta_par),
                                    (x_par - step_xy, y_par + step_y, theta_par),
                                    (x_par - step_xy, y_par - step_y, theta_par),
                                    (x_par, y_par, step_theta_p1),
                                    (x_par + step_xy, y_par, step_theta_p1),
                                    (x_par - step_xy, y_par, step_theta_p1),
                                    (x_par, y_par + step_y, step_theta_p1),
                                    (x_par, y_par - step_y, step_theta_p1),
                                    (x_par + step_xy, y_par + step_y, step_theta_p1),
                                    (x_par - step_xy, y_par - step_y, step_theta_p1),
                                    (x_par + step_xy, y_par - step_y, step_theta_p1),
                                    (x_par - step_xy, y_par + step_y, step_theta_p1),
                                    (x_par, y_par, step_theta_p2),
                                    (x_par + step_xy, y_par, step_theta_p2),
                                    (x_par - step_xy, y_par, step_theta_p2),
                                    (x_par, y_par + step_y, step_theta_p2),
                                    (x_par, y_par - step_y, step_theta_p2),
                                    (x_par + step_xy, y_par + step_y, step_theta_p2),
                                    (x_par - step_xy, y_par - step_y, step_theta_p2),
                                    (x_par + step_xy, y_par - step_y, step_theta_p2),
                                    (x_par - step_xy, y_par + step_y, step_theta_p2),))

            for i in range(len(state_temp)):
                if Iscollision(env, robot, state_temp[i]) == 1:
                        handles.append(env.plot3(points=array((state_temp[i][0], state_temp[i][1], 0.05)),
                                       pointsize = 10,
                                       colors=array(((1, 0, 0)))))
                        continue
                else:
                    if Isexist(state_temp[i], closedlist) == 0:  # does not exist
                        handles.append(env.plot3(points=array((state_temp[i][0], state_temp[i][1], 0.05)),
                                           pointsize = 10,
                                           colors=array(((0, 0, 1)))))
                        f_temp, pathcost_present = Totalcost(pathcost_par, state_temp[i], presentconfig, goalconfig)
                        id_newstate += 1
                        node_temp = Node(state_temp[i][0], state_temp[i][1], state_temp[i][2], id_newstate, id_par, pathcost_present)
                        q.put((f_temp, node_temp))
                        allnode.append(node_temp)
                        #print node_temp.x, node_temp.y, node_temp.theta, node_temp.id, node_temp.parentid
    if flag == 1:
        path = Creatpath(allnode, goalid, env, handles)
    else:
        path = []
        print 'There is no path'
    return path


