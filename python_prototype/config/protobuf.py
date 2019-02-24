"""@package config
Contains all config files necessary for simulator
Rignumber body configuration data

"""
import numpy as np
from enum import IntEnum
from queue import *
import copy

MAX_FLOAT = 9999999999
GRAPHICS_THREAD_INTERVAL_MS = 50
KINEMATICS_THREAD_INTERVAL_MS = 1
COM_THREAD_INTERVAL_MS = 1


class Point(object):
    __slots__ = ('x', 'y', 'z')

    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z


class Xfm(object):
    __slots__ = ('rxx', 'rxy', 'rxz', 'tx',
                 'ryx', 'ryy', 'ryz', 'ty',
                 'rzx', 'rzy', 'rzz', 'tz',
                 'm')

    def __init__(self,
                 rxx=1, rxy=0, rxz=0, tx=0,
                 ryx=0, ryy=1, ryz=0, ty=0,
                 rzx=0, rzy=0, rzz=1, tz=0):
        self.rxx = rxx
        self.rxy = rxy
        self.rxz = rxz
        self.tx = tx

        self.ryx = ryx
        self.ryy = ryy
        self.ryz = ryz
        self.ty = ty

        self.rzx = rzx
        self.rzy = rzy
        self.rzz = rzz
        self.tz = tz

        self.m = np.array([
            [self.rxx, self.rxy, self.rxz, self.tx],
            [self.ryx, self.ryy, self.ryz, self.ty],
            [self.rzx, self.rzy, self.rzz, self.tz],
            [0, 0, 0, 1]])

    def __mul__(self, another_xfm):
        m = np.dot(self.m, another_xfm.m)

        return Xfm(
            m[0, 0], m[0, 1], m[0, 2], m[0, 3],
            m[1, 0], m[1, 1], m[1, 2], m[1, 3],
            m[2, 0], m[2, 1], m[2, 2], m[2, 3])

    def inv(self):
        m = np.linalg.inv(self.m)
        return Xfm(
            m[0, 0], m[0, 1], m[0, 2], m[0, 3],
            m[1, 0], m[1, 1], m[1, 2], m[1, 3],
            m[2, 0], m[2, 1], m[2, 2], m[2, 3])

    def dot(self, p=Point()):
        v = np.dot(self.m, np.array([p.x, p.y, p.z, 1]))
        return [v[0], v[1], v[2]]


class Line(object):
    __slots__ = ('point_0', 'point_1')

    def __init__(self, point_0=Point(), point_1=Point()):
        self.point_0 = point_0
        self.point_1 = point_1


class JointType(IntEnum):
    UNKNOWN = -1
    REVOLUTE = 0
    PRISMATIC = 1


class Joint(object):
    __slots__ = ('index', 'name', 'xfm', 'type')

    def __init__(self, index=-1, name='', xfm=Xfm(), type=JointType.UNKNOWN):
        self.index = index
        self.name = name
        self.xfm = xfm
        self.type = type


class CadModel:
    def __init__(self, path='', xfm=Xfm(), color=[1, 1, 1], opacity=1):
        self.path = path
        self.xfm = xfm
        self.color = color
        self.opacity = opacity


class Graphics(object):
    def __init__(self,  cad_model=CadModel(), points=list(), lines=list(), points_size=10, points_color=[1, 1, 1],
                 lines_size=50, lines_color=[1, 1, 1]):
        self.cad_model = cad_model
        self.points = points
        self.points_size = points_size
        self.points_color = points_color
        self.lines = lines
        self.lines_size = lines_size
        self.lines_color = lines_color


class RigidBody(object):
    __slots__ = (
        'index', 'name', 'graphics', 'is_fixed', 'xfm', 'xfm_queue', 'joints', 'is_in_tree')

    def __init__(self, index=-1, name='', graphics=Graphics(), is_fixed=False, xfm=Xfm(), joints=list()):
        self.index = index
        self.name = name
        self.graphics = graphics
        self.is_fixed = is_fixed
        self.xfm = xfm
        self.xfm_queue = Queue()
        self.xfm_queue.put(copy.deepcopy(self.xfm))
        self.joints = joints


class Mate(object):
    __slots__ = ('index', 'name', 'value',
                 'bearing_rigid_body_index',
                 'bearing_joint_index',
                 'shaft_rigid_body_index',
                 'shaft_joint_index', 'is_limited', 'min', 'max', 'is_in_tree', 'angular_offset', 'linear_offset')

    def __init__(self,
                 index=-1, name='', value=0,
                 bearing_rigid_body_index=-1,
                 bearing_joint_index=-1,
                 shaft_rigid_body_index=-1,
                 shaft_joint_index=-1,
                 is_limited=False, minimum=-MAX_FLOAT, maximum=MAX_FLOAT, angular_offset=0, linear_offset=0):
        self.index = index
        self.name = name
        self.value = Queue()
        self.value.put(value)
        self.bearing_rigid_body_index = bearing_rigid_body_index
        self.bearing_joint_index = bearing_joint_index
        self.shaft_rigid_body_index = shaft_rigid_body_index
        self.shaft_joint_index = shaft_joint_index
        self.is_limited = is_limited
        self.min = minimum
        self.max = maximum
        self.angular_offset = angular_offset
        self.linear_offset = linear_offset

        # Inherited (Not available to users)
        self.is_in_tree = False


class RigidBodySystem(object):
    __slots__ = ('index', 'name', 'rigid_bodies', 'mates')

    def __init__(self, index=-1, name='', rigid_bodies=list(), mates=list()):
        self.index = index
        self.name = name
        self.rigid_bodies = rigid_bodies
        self.mates = mates


class Camera:
    def __init__(self, position=[0.0, 0.0, 1000.0], focal_point=[0, 0, 0], view_up=[0.0, 1.0, 0.0], clipping_range=None,
                 yaw=None, pitch=None, roll=None, azimuth=None, elevation=None):
        self.position = position
        self.focal_point = focal_point
        self.view_up = view_up
        self.clipping_range = clipping_range
        self.yaw = yaw
        self.pitch = pitch
        self.roll = roll
        self.azimuth = azimuth
        self.elevation = elevation



class Scene:
    def __init__(self, background_color=[0.0, 0.0, 0.0], is_gradient_on=True, window_size=[1500, 1400],
                 sim_view_port=[0.0, 0.0, 1.0, 1.0], camera=Camera(), is_full_screen=False, is_max_screen=False,
                 position=None):
        self.background_color = background_color
        self.is_gradient_on = is_gradient_on
        self.window_size = window_size
        self.sim_view_port = sim_view_port
        self.camera = camera
        self.is_full_screen = is_full_screen
        self.is_max_screen = is_max_screen
        self.position = position

