"""This component defines the joinery for roundwood elements with its surfaces and degrees of freedom."""
#! python3

import System
import typing

import Rhino
import Grasshopper

from ghpythonlib.componentbase import executingcomponent as component

import roundwood_joinery
import roundwoodJoineryBindings as rwj

import numpy as np

class RWJ_define_joinery(component):
    def RunScript(self,
            i_joinery: typing.List[rwj.JointGroup],
            i_point_cloud: Rhino.Geometry.PointCloud,
            i_reference_diameter: float,
            i_n_steps: int,
            i_epsilon: float) -> typing.List[rwj.JointGroup]:

        if i_n_steps is None:
            i_n_steps = 4
        if i_epsilon is None:
            i_epsilon = 1.0 

        beam_point_cloud = rwj.PointCloud(np.array([[pt.X, pt.Y, pt.Z] for pt in i_point_cloud.GetPoints()]))
        pc_skeleton = rwj.Utils.compute_point_cloud_skeleton(beam_point_cloud, i_reference_diameter/2, .01)
        rh_pc_skeleton = Rhino.Geometry.PointCloud([Rhino.Geometry.Point3d(pt[0], pt[1], pt[2]) for pt in pc_skeleton])
        beam = rwj.Beam(i_reference_diameter, i_joinery, pc_skeleton, beam_point_cloud)
        for joint_group in beam.get_joints_by_group():
            pt_on_skeleton = joint_group.get_joints()[0].get_closest_point_on_skeleton()
            for joints in joint_group.get_joints():
                for face in joints.get_faces():
                    print(f"face normal: {face.get_normal()}")
                    print(f"face current area: {face.compute_current_area(beam_point_cloud)}")
                    print(f"face target area: {face.get_target_area()}")

        transform_matrices = beam.compute_joint_group_optimisation(i_n_steps, i_epsilon)

        rh_transform_matrices = []
        for transform in transform_matrices:
            rh_transform = Rhino.Geometry.Transform()
            for i in range(4):
                for j in range(4):
                    rh_transform[i, j] = transform[i, j]
            rh_transform_matrices.append(rh_transform)

        return [beam, rh_transform_matrices]