"""This component optimizes the placement of joints for roundwood elements."""
#! python3

import System
import typing

import Rhino
import Grasshopper

from ghpythonlib.componentbase import executingcomponent as component

import roundwood_joinery
import roundwood_joinery.roundwoodJoineryBindings as rwj

import numpy as np

class RWJ_optimize_joint_placement(component):
    def RunScript(self,
            i_joinery: System.Collections.Generic.List[object],
            i_point_cloud: Rhino.Geometry.PointCloud,
            i_reference_diameter: float,
            i_n_steps: int,
            i_epsilon: float,
            i_path_save_file: str) -> typing.List[rwj.JointGroup]:

        if i_n_steps is None:
            i_n_steps = 4
        if i_epsilon is None:
            i_epsilon = 1.0 

        beam_point_cloud = rwj.PointCloud(np.array([[pt.X, pt.Y, pt.Z] for pt in i_point_cloud.GetPoints()]))
        pc_skeleton = rwj.Utils.compute_point_cloud_skeleton(beam_point_cloud, i_reference_diameter/2, .01)
        rh_pc_skeleton = Rhino.Geometry.PointCloud([Rhino.Geometry.Point3d(pt[0], pt[1], pt[2]) for pt in pc_skeleton])
        beam = rwj.Beam(i_reference_diameter, i_joinery, pc_skeleton, beam_point_cloud)

        if i_n_steps > 0:
            if i_path_save_file:
                transform_matrices = beam.compute_joint_group_optimisation(i_n_steps, i_epsilon, str(i_path_save_file))
            else:
                transform_matrices = beam.compute_joint_group_optimisation(i_n_steps, i_epsilon)

            rh_transform_matrices = []
            for transform in transform_matrices:
                rh_transform = Rhino.Geometry.Transform()
                for i in range(4):
                    for j in range(4):
                        rh_transform[i, j] = transform[i, j]
                rh_transform_matrices.append(rh_transform)
        else:
            rh_transform_matrices = [Rhino.Geometry.Transform(1)]
        return [beam, rh_transform_matrices]