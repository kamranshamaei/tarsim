"""@package config
Contains all config files necessary for simulator
Rigid body configuration data

"""
from config.protobuf import *

# Rigid Bodies ---------------------------------------------------------------------------------------------------------
# Single Joint Robot
# Base Link
points_0 = [Point(x=0, y=0, z=0), Point(x=-100, y=0, z=0), Point(x=100, y=0, z=0)]
lines_0 = [Line(point_0=points_0[1], point_1=points_0[2])]
graphics_0 = Graphics(points=points_0, lines=lines_0)
joints_0 = [Joint(type=JointType.REVOLUTE, index=0)]
rigid_body_0 = RigidBody(index=0, name='base', graphics=graphics_0, is_fixed=True, joints=joints_0)

# First Link
points_1 = [Point(x=-100, y=0, z=0), Point(x=100, y=0, z=0)]
lines_1 = [Line(point_0=points_1[0], point_1=points_1[1])]
graphics_1 = Graphics(points=points_0, lines=lines_1)
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

# Third Link
points_3 = [Point(x=-100, y=0, z=0), Point(x=100, y=0, z=0)]
lines_3 = [Line(point_0=points_3[0], point_1=points_3[1])]
graphics_3 = Graphics(points=points_3, lines=lines_3)
joints_3 = [Joint(type=JointType.REVOLUTE, index=0, xfm=Xfm(tx=-100)),
            Joint(type=JointType.REVOLUTE, index=1, xfm=Xfm(tx=100))]
rigid_body_3 = RigidBody(index=3, name='third', graphics=graphics_3, joints=joints_3)


# Fourth Link
points_4 = [Point(x=-100, y=0, z=0), Point(x=100, y=0, z=0)]
lines_4 = [Line(point_0=points_4[0], point_1=points_4[1])]
graphics_4 = Graphics(points=points_4, lines=lines_4)
joints_4 = [Joint(type=JointType.REVOLUTE, index=0, xfm=Xfm(tx=-100)),
            Joint(type=JointType.REVOLUTE, index=1, xfm=Xfm(tx=100))]
rigid_body_4 = RigidBody(index=4, name='fourth', graphics=graphics_4, joints=joints_4)


# Fifth Link
points_5 = [Point(x=-100, y=0, z=0), Point(x=100, y=0, z=0)]
lines_5 = [Line(point_0=points_5[0], point_1=points_5[1])]
graphics_5 = Graphics(points=points_5, lines=lines_5)
joints_5 = [Joint(type=JointType.REVOLUTE, index=0, xfm=Xfm(tx=-100)),
            Joint(type=JointType.REVOLUTE, index=1, xfm=Xfm(tx=100))]
rigid_body_5 = RigidBody(index=5, name='fifth', graphics=graphics_5, joints=joints_5)

# Sixth Link
points_6 = [Point(x=-100, y=0, z=0), Point(x=100, y=0, z=0)]
lines_6 = [Line(point_0=points_6[0], point_1=points_6[1])]
graphics_6 = Graphics(points=points_6, lines=lines_6)
joints_6 = [Joint(type=JointType.REVOLUTE, index=0, xfm=Xfm(tx=-100)),
            Joint(type=JointType.REVOLUTE, index=1, xfm=Xfm(tx=100))]
rigid_body_6 = RigidBody(index=6, name='sixth', graphics=graphics_6, joints=joints_6)

# Seventh Link
points_7 = [Point(x=-100, y=0, z=0), Point(x=100, y=0, z=0)]
lines_7 = [Line(point_0=points_1[0], point_1=points_7[1])]
graphics_7 = Graphics(points=points_7, lines=lines_7)
joints_7 = [Joint(type=JointType.REVOLUTE, index=0, xfm=Xfm(tx=-100)),
            Joint(type=JointType.REVOLUTE, index=1, xfm=Xfm(tx=100))]
rigid_body_7 = RigidBody(index=7, name='seventh', graphics=graphics_7, joints=joints_7)

# Eight Link
points_8 = [Point(x=-100, y=0, z=0), Point(x=100, y=0, z=0)]
lines_8 = [Line(point_0=points_8[0], point_1=points_8[1])]
graphics_8 = Graphics(points=points_8, lines=lines_8)
joints_8 = [Joint(type=JointType.REVOLUTE, index=0, xfm=Xfm(tx=-100)),
            Joint(type=JointType.REVOLUTE, index=1, xfm=Xfm(tx=100))]
rigid_body_8 = RigidBody(index=8, name='eight', graphics=graphics_8, joints=joints_8)
# Rigid Body System ----------------------------------------------------------------------------------------------------
rigid_bodies = [rigid_body_0, rigid_body_1, rigid_body_2, rigid_body_3,
                rigid_body_4, rigid_body_5, rigid_body_6, rigid_body_7, rigid_body_8]
mates = [
    Mate(index=0,
         bearing_rigid_body_index=0, bearing_joint_index=0,
         shaft_rigid_body_index=1, shaft_joint_index=0),
    Mate(index=1,
         bearing_rigid_body_index=1, bearing_joint_index=1,
         shaft_rigid_body_index=2, shaft_joint_index=0),
    Mate(index=2,
         bearing_rigid_body_index=1, bearing_joint_index=1,
         shaft_rigid_body_index=3, shaft_joint_index=0),
    Mate(index=3,
         bearing_rigid_body_index=3, bearing_joint_index=1,
         shaft_rigid_body_index=4, shaft_joint_index=0),
    Mate(index=4,
         bearing_rigid_body_index=2, bearing_joint_index=1,
         shaft_rigid_body_index=5, shaft_joint_index=0),
    Mate(index=5,
         bearing_rigid_body_index=3, bearing_joint_index=1,
         shaft_rigid_body_index=6, shaft_joint_index=0),
    Mate(index=6,
         bearing_rigid_body_index=2, bearing_joint_index=1,
         shaft_rigid_body_index=7, shaft_joint_index=0),
    Mate(index=7,
         bearing_rigid_body_index=8, bearing_joint_index=1,
         shaft_rigid_body_index=0, shaft_joint_index=0)]


sample_robot = RigidBodySystem(index=0, name='robot', rigid_bodies=rigid_bodies, mates=mates)
