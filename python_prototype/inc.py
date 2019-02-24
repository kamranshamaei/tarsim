""""@package config
Contains all config files necessary for simulator
Rignumber body configuration data

"""
import copy
import time
import inspect
from config.protobuf import *
import math


def set_inter_thread_data(new_data, data, data_lock):
    data_lock.acquire()
    if len(new_data) is len(data):
        for i in range(len(new_data)):
            data[i] = new_data[i]
    data_lock.release()


def get_inter_thread_data(data, data_lock):
    data_lock.acquire()
    data_copy = copy.deepcopy(data)
    data_lock.release()
    return data_copy


IS_ACCURATE_TIMER = False


def custom_sleep(sec):
    target_time = time.clock() + sec
    while time.clock() < target_time:
        if IS_ACCURATE_TIMER:
            pass
        else:
            time.sleep(0.00001)


class TimeRegulator:
    def __init__(self, interval):
        self.desired_interval = interval
        self.tic = time.clock()
        self.toc = time.clock()

    def regulate_time(self):
        self.toc = time.clock()
        actual_interval = self.toc - self.tic
        if actual_interval < self.desired_interval:
            custom_sleep(self.desired_interval - actual_interval)
        final_interval = time.clock() - self.tic
        self.tic = time.clock()
        return final_interval

    def set_desired_interval(self, interval):
        self.desired_interval = interval


U_X = np.array([1, 0, 0])
U_Y = np.array([0, 1, 0])
U_Z = np.array([0, 0, 1])


def rad(degree):
    return math.radians(degree)


def log(msg):
    file_name = (str(inspect.stack()[1][1]).split('/')[-1:][0]).ljust(20)
    line_number = (str(inspect.stack()[1][2])).ljust(5)
    function_name = (inspect.stack()[1][3]).ljust(20)
    address = ("| File: " + file_name + "| Line: " + line_number + "| Function: " + function_name)
    print(msg.ljust(50) + address)


class Node:
    def __init__(self, parent=None, rigid_body=RigidBody(), mate_to_parent=Mate()):
        self.name = str(rigid_body.index)
        self.parent = parent
        self.children = list()
        self.rigid_body = rigid_body
        self.xfm_jn_n = None
        self.xfm_m_jm = None
        self.joint_type = None
        self.mate_to_parent = mate_to_parent
        self.rigid_body_actors = None

        if self.parent is not None:
            # If the current node is the bearing and the parent node is the shaft
            if (mate_to_parent.bearing_rigid_body_index is self.rigid_body.index) and\
               (mate_to_parent.shaft_rigid_body_index is self.parent.rigid_body.index):

                # Find the associated bearing joint among the current rigid body joints
                for i in range(len(self.rigid_body.joints)):
                    if self.mate_to_parent.bearing_joint_index is self.rigid_body.joints[i].index:
                        self.xfm_jn_n = self.rigid_body.joints[i].xfm.inv()
                        self.joint_type = self.rigid_body.joints[i].type

                # Find the associated shaft joint among the parent's rigid body joints
                for i in range(len(self.parent.rigid_body.joints)):
                    if self.mate_to_parent.shaft_joint_index is self.parent.rigid_body.joints[i].index:
                        self.xfm_m_jm = self.parent.rigid_body.joints[i].xfm
                        if self.joint_type is not self.parent.rigid_body.joints[i].type:
                            log("Joint types are inconsistent")
                            raise ValueError

            # If the current node is the shaft and the parent node is the bearing
            elif (mate_to_parent.shaft_rigid_body_index is self.rigid_body.index) and\
                 (mate_to_parent.bearing_rigid_body_index is self.parent.rigid_body.index):

                # Find the associated shaft joint among the current rigid body joints
                for i in range(len(self.rigid_body.joints)):
                    if self.mate_to_parent.shaft_joint_index is self.rigid_body.joints[i].index:
                        self.xfm_jn_n = self.rigid_body.joints[i].xfm.inv()
                        self.joint_type = self.rigid_body.joints[i].type

                # Find the associated bearing joint among the parent's rigid body joints
                for i in range(len(self.parent.rigid_body.joints)):
                    if self.mate_to_parent.bearing_joint_index is self.rigid_body.joints[i].index:
                        self.xfm_m_jm = self.parent.rigid_body.joints[i].xfm
                        if self.joint_type is not self.parent.rigid_body.joints[i].type:
                            log("Joint types are inconsistent")
                            raise ValueError
            else:
                log("Mate indices are incorrect")
                raise ValueError

            if (self.xfm_jn_n is None) or (self.xfm_m_jm is None):
                log("Invalid joint assignment for node: %s" % self.name)
                raise ValueError

            if (self.joint_type is None) or (self.joint_type is JointType.UNKNOWN):
                log("Invalid joint type for node: %s" % self.name)
                raise ValueError

        if parent:
            self.parent.children.append(self)


