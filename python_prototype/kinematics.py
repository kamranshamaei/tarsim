"""@package config
Contains all config files necessary for simulator
Rignumber body configuration data

"""
from inc import *
import transformations as tf
import threading


KINEMATICS_THREAD_INTERVAL_S = KINEMATICS_THREAD_INTERVAL_MS / 1000


def xfm_revolute_joint(value, angular_offset, linear_offset):
    m = np.dot(tf.rotation_matrix(rad(value + angular_offset), U_Z), tf.translation_matrix(linear_offset * U_Z))
    return Xfm(
                m[0, 0], m[0, 1], m[0, 2], m[0, 3],
                m[1, 0], m[1, 1], m[1, 2], m[1, 3],
                m[2, 0], m[2, 1], m[2, 2], m[2, 3])


def xfm_prismatic_joint(value, angular_offset, linear_offset):
    m = np.dot(tf.rotation_matrix(rad(angular_offset), U_Z),
               tf.translation_matrix((value + linear_offset) * U_Z))
    return Xfm(
                m[0, 0], m[0, 1], m[0, 2], m[0, 3],
                m[1, 0], m[1, 1], m[1, 2], m[1, 3],
                m[2, 0], m[2, 1], m[2, 2], m[2, 3])


def calculate_node_to_parent_xfm(node=Node()):
    value = 0
    for i in range(node.mate_to_parent.value.qsize()):
        value = node.mate_to_parent.value.get()

    if node.joint_type is JointType.REVOLUTE:
        xfm_jm_jn = xfm_revolute_joint(
            value, node.mate_to_parent.angular_offset, node.mate_to_parent.linear_offset)
    elif node.joint_type is JointType.PRISMATIC:
        xfm_jm_jn = xfm_prismatic_joint(
            value, node.mate_to_parent.angular_offset, node.mate_to_parent.linear_offset)
    else:
        xfm_jm_jn = Xfm()
    xfm_m_n = node.xfm_m_jm * xfm_jm_jn * node.xfm_jn_n

    return xfm_m_n


class Kinematics:
    def __init__(self, root=Node(), joint_index_dict=dict(), joint_value_dict=dict()):
        self.root = root
        self.joint_index_dict = joint_index_dict
        self.joint_value_dict = joint_value_dict

        self.kinematics_thread = threading.Thread(target=self.kinematics_thread_function)
        self.kinematics_thread_time_regulator = TimeRegulator(KINEMATICS_THREAD_INTERVAL_S)
        self.kinematics_thread_time_interval = 0
        self.kinematics_thread.start()

    def kinematics_thread_function(self):
        while True:
            if not self.calculate_xfm_rigid_bodies_to_world():
                log("Failed to calculate xfms of rigid bodies")
            self.kinematics_thread_time_interval = self.kinematics_thread_time_regulator.regulate_time()

    def set_joint_values(self, joint_value_dict):
        if (len(joint_value_dict) is not len(self.joint_value_dict)) or\
                (len(joint_value_dict) is not len(self.joint_index_dict)):
            log("Joint values size mismatch")
            return False

        for key in joint_value_dict:
            if (key not in self.joint_index_dict) or ((key not in self.joint_value_dict)):
                log("Joint key not in dictionary")
                return False

            self.joint_index_dict[key].mate_to_parent.value.put(joint_value_dict[key])

    def calculate_xfm_rigid_bodies_to_world(self):
        if not self.calculate_children_poses(self.root):
            log("Failed to parse tree xfms")
            return False
        return True

    def calculate_children_poses(self, node=Node()):
        if node is self.root:
            if node.rigid_body.xfm is None:
                log("Root xfm is None")
                return False
        else:

            # Calculate the xfm wrt the parent frame
            if not node.mate_to_parent.value.empty():
                xfm_m_n = calculate_node_to_parent_xfm(node)
                node.rigid_body.xfm = node.parent.rigid_body.xfm * xfm_m_n
                node.rigid_body.xfm_queue.put(copy.deepcopy(node.rigid_body.xfm))



        if node.children:
            for i in range(len(node.children)):
                if not self.calculate_children_poses(node.children[i]):
                    log("Failed to parse tree xfm for node %s" % node.children[i].name)
                    return False

        return True
