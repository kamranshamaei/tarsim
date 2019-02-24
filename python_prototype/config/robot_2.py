"""@package config
Contains all config files necessary for simulator
Rigid body configuration data

"""
from config.protobuf import *

# Rigid Bodies ---------------------------------------------------------------------------------------------------------
# Single Joint Robot
# Base Link
points_0 = [Point(x=0, y=0, z=0), Point(x=-100, y=0, z=0), Point(x=100, y=0, z=0), Point(x=0, y=100, z=0)]
lines_0 = [Line(point_0=points_0[1], point_1=points_0[2]),
           Line(point_0=points_0[0], point_1=points_0[3])]
graphics_0 = Graphics(points=points_0, lines=lines_0)
joints_0 = [Joint(type=JointType.REVOLUTE, index=0)]
rigid_body_0 = RigidBody(index=0, name='base', graphics=graphics_0, is_fixed=True, joints=joints_0)

# First Link
points_1 = [Point(x=-100, y=0, z=0), Point(x=100, y=0, z=0)]
lines_1 = [Line(point_0=points_1[0], point_1=points_1[1])]
cad_1_xfm = Xfm()
cad_1_xfm.m = np.array([[ 0, -1,  0,  0],
                        [ 1,  0,  0,  0],
                        [ 0,  0,  1,  0],
                        [ 0,  0,  0,  1]])
cad_1 = CadModel(path='/home/kamran/eit/KinSimulator/python/config/link.stl', xfm=cad_1_xfm)
graphics_1 = Graphics(points=points_1, lines=lines_1, cad_model=cad_1, lines_color=[0, 0.5, 0.4])
joints_1 = [Joint(type=JointType.REVOLUTE, index=0, xfm=Xfm(tx=-100)),
            Joint(type=JointType.REVOLUTE, index=1, xfm=Xfm(tx=100))]
rigid_body_1 = RigidBody(index=1, name='first', graphics=graphics_1, joints=joints_1)

# Second Link
points_2 = [Point(x=-100, y=0, z=0), Point(x=100, y=0, z=0)]
lines_2 = [Line(point_0=points_2[0], point_1=points_2[1])]
graphics_2 = Graphics(points=points_2, lines=lines_2)
joints_2 = [Joint(type=JointType.REVOLUTE, index=0, xfm=Xfm(tx=-100)),
            Joint(type=JointType.REVOLUTE, index=1, xfm=Xfm(tx=100))]
rigid_body_2 = RigidBody(index=2, name='second', graphics=graphics_2, joints=joints_2)

# Rigid Body System ----------------------------------------------------------------------------------------------------
rigid_bodies = [rigid_body_0, rigid_body_1, rigid_body_2]
mates = [
    Mate(index=0,
         bearing_rigid_body_index=0, bearing_joint_index=0,
         shaft_rigid_body_index=1, shaft_joint_index=0),
    Mate(index=1,
         bearing_rigid_body_index=1, bearing_joint_index=1,
         shaft_rigid_body_index=2, shaft_joint_index=0)]


sample_robot = RigidBodySystem(index=0, name='robot', rigid_bodies=rigid_bodies, mates=mates)
camera = Camera(position=[0.0, 0.0, 2000.0])
scene = Scene(camera=camera, position=[400, 100], window_size=[600, 600])
