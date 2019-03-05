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
    def __init__(self, _input, id_in, parentid_in):
        self.config = _input
        self.id = id_in
        self.parentid = parentid_in

    def printme(self):
        print self.config[0], self.config[1], self.config[2], self.config[3], self.config[4], self.config[5], self.id, self.parentid

def creatrandomdir(presentconfig, randomvector, step):
    _randomvector = array(randomvector)
    _presentconfig = array(presentconfig)
    randdir = step*(_randomvector-_presentconfig)/numpy.linalg.norm(_randomvector-_presentconfig)
    return randdir

def rotaionnorm(config1, config2):
    return numpy.linalg.norm(matrix(config1)-matrix(config2))

def getclosenode(randconfig, allnode):
    dis_min = rotaionnorm(allnode[0].config, randconfig)
    index_min = 0
    for i in range(len(allnode)):
        if abs(allnode[i].config[4]-randconfig[4]) > dis_min or abs(allnode[i].config[1]-randconfig[1]) > dis_min:
            continue
        dis_temp = numpy.linalg.norm(matrix(allnode[i].config)-matrix(randconfig))
        if dis_temp < dis_min:
            dis_min = dis_temp
            index_min = i

    return index_min

def Iscollision(env, robot, newconfig):
    robot.SetActiveDOFValues(newconfig)
    if env.CheckCollision(robot) is False:
        return 0
    else:
        return 1

def IsTemination(presentconfig, goalconfig, step):
    if rotaionnorm(presentconfig, goalconfig) < step:
        return 1
    return 0

def Creatpath(allnode, goalid, env, handles, robot):
    path = []
    idpresent = goalid
    while True:
        path.append(allnode[idpresent].config.tolist())
        Plotendeffector(handles, env, robot, allnode[idpresent].config, 1)
        if idpresent == 0:
            break
        else:
            idpresent = allnode[idpresent].parentid
    path.reverse()
    return path

def GetEETransform(robot, activedofvalues=None):
    if activedofvalues != None:
        robot.SetActiveDOFValues(activedofvalues);
    manip = robot.GetActiveManipulator()
    return manip.GetEndEffectorTransform()

def Plotendeffector(handles, env, robot, activedofvalues, choise):
    size = 5
    transform = GetEETransform(robot, activedofvalues)
    if choise == 0:
        col = array(((0, 0, 1)))
        #size = 15
    elif choise == 1:
        col = array(((1, 0, 0)))
    elif choise == 2:
        col = array(((0, 1, 0)))
    else:
        col = array(((0, 0, 0)))
    handles.append(env.plot3(points=array((transform[0][3], transform[1][3], transform[2][3])),
                       pointsize = size,
                       colors=col))

def Randconfig(tar_ratio, limit, goalconfig):
    if random.random() <= tar_ratio*1:
            randconfig = array(goalconfig)
    else:
            randconfig = array([random.uniform(limit[0][0], limit[0][1]), random.uniform(limit[1][0], limit[1][1]),
                                random.uniform(limit[2][0], limit[2][1]), random.uniform(limit[3][0], limit[3][1]),
                                random.uniform(-2*pi, 0), random.uniform(limit[5][0], limit[5][1])])
            if random.random() <= tar_ratio:
                randconfig = array(goalconfig)+0.3*array([random.uniform(limit[0][0]-goalconfig[0], limit[0][1]-goalconfig[0]),
                                    random.uniform(limit[1][0]-goalconfig[1], limit[1][1]-goalconfig[1]),
                                    random.uniform(limit[2][0]-goalconfig[2], limit[2][1]-goalconfig[2]),
                                    random.uniform(limit[3][0]-goalconfig[3], limit[3][1]-goalconfig[3]),
                                    random.uniform(-pi-goalconfig[4], pi-goalconfig[4]),
                                    random.uniform(limit[5][0]-goalconfig[5], limit[5][1]-goalconfig[5])])
    return randconfig

def Smoothing(path_unsmooth, env, robot, ite_times, handles):
    path_smooth = deepcopy(path_unsmooth)
    for j in range(ite_times):
        index1 = random.randint(0, len(path_smooth))
        index2 = random.randint(0, len(path_smooth))
        if abs(index2-index1) < 2:
            continue
        dir = array(deepcopy(path_smooth[index1]))-array(deepcopy(path_smooth[index2]))
        step_smooth = 0.05*dir/numpy.linalg.norm(dir)
        presentconfig = array(deepcopy(path_smooth[index2])) + step_smooth
        flag_coll_free = 1
        while numpy.linalg.norm(presentconfig-array(path_smooth[index1])) > 0.05:
            if Iscollision(env, robot, presentconfig) == 1:
                flag_coll_free = 0
                break
            presentconfig += step_smooth

        if flag_coll_free == 1:
            del1 = min(index1, index2)
            del2 = max(index1, index2)
            del path_smooth[del1+1:del2]

        dir = array(deepcopy(path_smooth[del1+1]))-array(deepcopy(path_smooth[del1]))
        step_smooth = 0.05*dir/numpy.linalg.norm(dir)
        presentconfig = array(deepcopy(path_smooth[del1])) + step_smooth
        end_config = path_smooth[del1+1]
        while numpy.linalg.norm(presentconfig-array(end_config)) > 0.05:
            del1 = del1 + 1
            path_smooth.insert(del1, presentconfig.tolist())
            presentconfig = presentconfig + step_smooth
    for i in range(len(path_smooth)):
        Plotendeffector(handles, env, robot, path_smooth[i], 0)
    return path_smooth


def RRTmainloop(env, robot, goalconfig, initialconfig, step, tar_ratio, handles, limit, ite_times):
    allnode = []
    allnode.append(Node(array(deepcopy(initialconfig)), 0, 0))
    id_presnt = 0
    flag_terminate = 0
    Plotendeffector(handles, env, robot, goalconfig, 1)
    while True:
        randconfig = Randconfig(tar_ratio, limit, goalconfig)
        id_par = getclosenode(randconfig, allnode)
        presentconfig = deepcopy(allnode[id_par].config)
        #Plotendeffector(handles, env, robot, randconfig.tolist(), 3)
        step_present = creatrandomdir(presentconfig, randconfig, step)
        while True:
            presentconfig = presentconfig + step_present
            if Iscollision(env, robot, presentconfig) == 0:
                id_presnt += 1
                allnode.append(Node(deepcopy(presentconfig), id_presnt, id_par))
                #if mod(id_presnt, 200) == 0:
                #    print id_presnt
                id_par = id_presnt
                #Plotendeffector(handles, env, robot, presentconfig, 0)
                if IsTemination(presentconfig, goalconfig, step) == 1:
                    goalid = id_presnt
                    flag_terminate = 1
                    break
                if IsTemination(presentconfig, randconfig, step) == 1:
                    break
            else:
                break
        if flag_terminate == 1:
            break
    path_unsmooth = Creatpath(allnode, goalid, env, handles, robot)
    path_smooth = Smoothing(path_unsmooth, env, robot, ite_times, handles)
    return path_smooth




