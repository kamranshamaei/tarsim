try:
    import vtk
except:
    print("Cannot import vtk")

import copy
import numpy as np
from inc import *


def create_cad_actor(cad_model=CadModel(), xfm=Xfm()):
    actor = None
    if cad_model.path:
        stl_reader = vtk.vtkSTLReader()
        stl_reader.SetFileName(cad_model.path)

        stl_normals = vtk.vtkPolyDataNormals()
        stl_normals.SetFeatureAngle(160)
        stl_normals.SetInputConnection(stl_reader.GetOutputPort())

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(stl_normals.GetOutputPort())

        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(cad_model.color)
        actor.GetProperty().SetOpacity(cad_model.opacity)
        actor.GetProperty().SetInterpolationToGouraud()
        t = vtk.vtkTransform()
        t.SetMatrix([
            xfm.rxx, xfm.rxy, xfm.rxz, xfm.tx,
            xfm.ryx, xfm.ryy, xfm.ryz, xfm.ty,
            xfm.rzx, xfm.rzy, xfm.rzz, xfm.tz,
            0, 0, 0, 1])
        actor.SetUserTransform(t)
    return actor


def create_lines_actors(lines=list(), xfm=Xfm(), lines_size=10, lines_color=[1, 1, 1]):
    actor = None
    if lines:
        # Create the polydata where we will store all the geometric data
        linesPolyData = vtk.vtkPolyData()

        # Create a vtkPoints container and store the points in it
        pts = vtk.vtkPoints()

        # Add the points to the polydata container
        linesPolyData.SetPoints(pts)

        # Create a vtkCellArray container and store the lines in it
        vtk_lines = vtk.vtkCellArray()

        # Add the lines to the polydata container
        linesPolyData.SetLines(vtk_lines)

        for i in range(len(lines)):
            ln = lines[i]

            # Create two points for each line
            p0 = xfm.dot(ln.point_0)
            p1 = xfm.dot(ln.point_1)

            pts.InsertNextPoint(p0)
            pts.InsertNextPoint(p1)

            # Create the ith line (between P0 and P1)
            line = vtk.vtkLine()
            line.GetPointIds().SetId(0, 2 * i)  # the second is the index of P0 in linesPolyData's points
            line.GetPointIds().SetId(1, 2 * i + 1)  # the second is the index of P1 in linesPolyData's points

            vtk_lines.InsertNextCell(line)

        # Setup the visualization pipeline
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(linesPolyData)

        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(lines_color)
        actor.GetProperty().SetLineWidth(lines_size)

    return actor


def create_points_actors(points=list(), xfm=Xfm(), points_size=10, points_color=[1, 1, 1]):
    actor = list()
    if points:
        for i in range(len(points)):
            sphere_source = vtk.vtkSphereSource()
            p = xfm.dot(points[i])
            sphere_source.SetCenter(p[0], p[1], p[2])
            sphere_source.SetRadius(points_size)
            sphere_source.Update()

            mapper = vtk.vtkPolyDataMapper()
            mapper.SetInputConnection(sphere_source.GetOutputPort())

            actor.append(vtk.vtkActor())
            actor[i].SetMapper(mapper)

    return actor


def create_camera(camera=Camera()):
    vtk_camera = vtk.vtkCamera()
    vtk_camera.SetPosition(camera.position)
    vtk_camera.SetFocalPoint(camera.focal_point)
    vtk_camera.SetViewUp(camera.view_up)

    if camera.clipping_range is not None:
        vtk_camera.SetClippingRange(camera.clipping_range)
    if camera.yaw is not None:
        vtk_camera.Yaw(camera.yaw)
    if camera.pitch is not None:
        vtk_camera.Pitch(camera.pitch)
    if camera.roll is not None:
        vtk_camera.Roll(camera.roll)
    if camera.azimuth is not None:
        vtk_camera.Azimuth(camera.azimuth)
    if camera.elevation is not None:
        vtk_camera.Elevation(camera.elevation)
    return vtk_camera



class RigidBodyActors:
    def __init__(self, xfm=Xfm(), points=list(), lines=list(), cad_model=CadModel(), points_size=10,
                 points_color=[1, 1, 1], lines_size=10, lines_color=[1, 1, 1]):
        self.mutex_xfm = vtk.vtkMutexLock()
        self.mutex_xfm.Lock()
        self.xfm = copy.deepcopy(xfm)
        self.mutex_xfm.Unlock()

        self.points = points
        self.lines = lines
        self.lines_size = lines_size
        self.cad_model_xfm = cad_model.xfm

        self.points_actors = \
            create_points_actors(xfm=xfm, points=points, points_size=points_size, points_color=points_color)
        self.lines_actors = create_lines_actors(xfm=xfm, lines=self.lines, lines_size=lines_size, lines_color=lines_color)
        self.cad_model_actor = create_cad_actor(cad_model=cad_model, xfm=xfm)

    def update(self, xfm):
        self.xfm = xfm

        if self.lines_actors is not None:
            self.update_lines_actors(self.lines_actors, xfm)
        if self.points_actors is not None:
            self.update_points_actors(self.points_actors, xfm)
        if self.cad_model_actor is not None:
            self.update_cad_actor(xfm)

    def update_points_actors(self, actor=list(), xfm=Xfm()):
        for i in range(len(actor)):
            sphere_source = actor[i].GetMapper().GetInputConnection(0, 0).GetProducer()
            p = xfm.dot(self.points[i])
            sphere_source.SetCenter(p[0], p[1], p[2])

    def update_lines_actors(self, actor=vtk.vtkActor(), xfm=Xfm()):
        for i in range(len(self.lines)):
            ln = self.lines[i]
            # Create two points for each line
            p0 = xfm.dot(ln.point_0)
            p1 = xfm.dot(ln.point_1)

            # Get the current point
            actor.GetMapper().GetInput().GetPoints().SetPoint(2 * i, p0)
            actor.GetMapper().GetInput().GetPoints().SetPoint(2 * i + 1, p1)
            actor.GetMapper().GetInput().GetPoints().Modified()

    def update_cad_actor(self, xfm=Xfm()):
        if self.cad_model_actor is not None:
            new_xfm = xfm * self.cad_model_xfm
            t = vtk.vtkTransform()
            t.SetMatrix([
                new_xfm.rxx, new_xfm.rxy, new_xfm.rxz, new_xfm.tx,
                new_xfm.ryx, new_xfm.ryy, new_xfm.ryz, new_xfm.ty,
                new_xfm.rzx, new_xfm.rzy, new_xfm.rzz, new_xfm.tz,
                0, 0, 0, 1])
            self.cad_model_actor.SetUserTransform(t)


class RigidBodySystemScene:
    def __init__(self, scene=Scene(), root=Node()):
        self.renderer = vtk.vtkRenderer()
        self.renderer.SetBackground(scene.background_color[0], scene.background_color[1], scene.background_color[2])
        self.renderer.SetViewport(
            scene.sim_view_port[0], scene.sim_view_port[1], scene.sim_view_port[2], scene.sim_view_port[3])

        if scene.is_gradient_on:
            self.renderer.GradientBackgroundOn()
        else:
            self.renderer.GradientBackgroundOff()

        self.camera = create_camera(scene.camera)
        self.renderer.SetActiveCamera(self.camera)

        self.add_tree_lines_actors(node=root)
        self.add_tree_points_actors(node=root)
        self.add_tree_cad_models(node=root)

    def add_tree_lines_actors(self, node=Node()):
        if node.rigid_body_actors.lines_actors:
            self.renderer.AddActor(node.rigid_body_actors.lines_actors)
        if node.children:
            for i in range(len(node.children)):
                self.add_tree_lines_actors(node=node.children[i])

    def add_tree_points_actors(self, node=Node()):
        if node.rigid_body_actors.lines_actors:
            for i in range(len(node.rigid_body_actors.points_actors)):
                self.renderer.AddActor(node.rigid_body_actors.points_actors[i])
        if node.children:
            for i in range(len(node.children)):
                self.add_tree_points_actors(node=node.children[i])

    def add_tree_cad_models(self, node=Node()):
        if node.rigid_body_actors.cad_model_actor:
            self.renderer.AddActor(node.rigid_body_actors.cad_model_actor)
        if node.children:
            for i in range(len(node.children)):
                self.add_tree_cad_models(node=node.children[i])


class Simulator:
    def __init__(self, scene=Scene(), root=Node()):
        self.root = root
        self.create_rigid_body_actors(self.root)

        self.scene = RigidBodySystemScene(scene=scene, root=self.root)

    def create_rigid_body_actors(self, current_node=Node()):
        current_node.rigid_body_actors = RigidBodyActors(
            xfm=current_node.rigid_body.xfm_queue.get(),
            points=current_node.rigid_body.graphics.points,
            lines=current_node.rigid_body.graphics.lines,
            cad_model=current_node.rigid_body.graphics.cad_model,
            points_size=current_node.rigid_body.graphics.points_size,
            points_color=current_node.rigid_body.graphics.points_color,
            lines_size = current_node.rigid_body.graphics.lines_size,
            lines_color=current_node.rigid_body.graphics.lines_color)
        if current_node.children:
            for i in range(len(current_node.children)):
                self.create_rigid_body_actors(current_node=current_node.children[i])

    def update(self):
        self.update_node_actors(node=self.root)

    def update_node_actors(self, node=Node()):
        xfm = None
        for i in range(node.rigid_body.xfm_queue.qsize()):
            xfm = node.rigid_body.xfm_queue.get()
        if xfm is not None:
            node.rigid_body_actors.update(xfm)
        if node.children:
            for i in range(len(node.children)):
                self.update_node_actors(node=node.children[i])


class Window:
    def __init__(self, root=Node(), scene=Scene()):
        self.channels = list()
        self.channels.append(Simulator(scene=scene, root=root))

        # Render window ------------------------------------------------------------------------------------------------
        self.render_window = vtk.vtkRenderWindow()
        for i in range(len(self.channels)):
            self.render_window.AddRenderer(self.channels[i].scene.renderer)

        # Create a render window ---------------------------------------------------------------------------------------
        if scene.is_full_screen:
            self.render_window.SetFullScreen(1)
        elif scene.is_max_screen:
            width, height = self.render_window.GetScreenSize()
            self.render_window.SetSize(width, height)
        else:
            self.render_window.SetSize(scene.window_size[0], scene.window_size[1])

        if scene.position is not None:
            self.render_window.SetPosition(scene.position[0], scene.position[1])

    def update(self):
        for i in range(len(self.channels)):
            self.channels[i].update()


class Gui:
    def __init__(self, root=Node(), scene=Scene()):
        try:
            self.window = Window(root=root, scene=scene)
        except:
            raise Exception('Gui Error!', 'Window')

        # Create a render window interactor ----------------------------------------------------------------------------
        self.render_window_interactor = vtk.vtkRenderWindowInteractor()
        self.render_window_interactor.SetRenderWindow(self.window.render_window)
        self.render_window_interactor.Initialize()
        self.render_window_interactor.AddObserver('TimerEvent', self.update)
        self.render_window_interactor.AddObserver('ExitEvent', self.stop)
        self.render_window_interactor.CreateRepeatingTimer(GRAPHICS_THREAD_INTERVAL_MS)
        self.render_window_interactor.Start()

    def stop(self, obj, event):
        self.render_window_interactor.DestroyTimer()
        self.render_window_interactor.GetRenderWindow().Finalize()
        self.render_window_interactor.TerminateApp()
        self.render_window_interactor = None
        self.window = None

    def update(self, obj, event):
        self.window.update()
        self.render_window_interactor.GetRenderWindow().Render()
