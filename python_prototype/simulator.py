"""@package config
Contains all config files necessary for simulator
Rignumber body configuration data

"""
from kinematics import Kinematics
import threading
from inc import *
from gui import Gui
from communication import ComminucationModule
from rigid_body_system_parser import RigidBodySystemParser


class Simulator:
    def __init__(self, rigid_body_system=RigidBodySystem(), scene=Scene()):
        self.rigid_body_system = rigid_body_system

        try:
            self.rigid_body_system_parser = RigidBodySystemParser(self.rigid_body_system)

            self.kinematics = Kinematics(
                root=self.rigid_body_system_parser.get_tree(),
                joint_index_dict=self.rigid_body_system_parser.get_joint_index_dict(),
                joint_value_dict=self.rigid_body_system_parser.get_joint_value_dict())

            self.com = ComminucationModule(
                joint_index_dict=self.rigid_body_system_parser.get_joint_index_dict(),
                joint_value_dict=self.rigid_body_system_parser.get_joint_value_dict())

            self.gui = Gui(root=self.rigid_body_system_parser.get_tree(), scene=scene)

        except Exception as error:
            log('Error: ' + repr(error))
            raise Exception('Simulator Error!', 'Simulator')
