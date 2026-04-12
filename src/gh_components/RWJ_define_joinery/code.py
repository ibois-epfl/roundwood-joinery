"""This component defines the joinery for roundwood elements with its surfaces and degrees of freedom."""
#! python3

import System
import typing

import Rhino
import Grasshopper

import ghpythonlib.treehelpers as th
from ghpythonlib.componentbase import executingcomponent as component

import roundwood_joinery
import roundwoodJoineryBindings as rwj

import numpy as np

class RWJ_define_joinery(component):
    def RunScript(self,
            i_curves_datatree: Grasshopper.DataTree[Rhino.Geometry.Polyline],
            i_dof_vectors: Grasshopper.DataTree[Rhino.Geometry.Vector3d],
            i_target_areas_datatree: Grasshopper.DataTree[float]) -> typing.List[rwj.JointGroup]:
        curve_lists = th.tree_to_list(i_curves_datatree)
        target_areas = th.tree_to_list(i_target_areas_datatree)
        dof_lists = th.tree_to_list(i_dof_vectors)
        joint_groups = []
        for i, crv_group in enumerate(curve_lists):
            joint_group = []
            for crv in crv_group:
                corner_pts = [np.array([pt.X, pt.Y, pt.Z]) for pt in crv]
                v1 = corner_pts[1] - corner_pts[0]
                v2 = corner_pts[2] - corner_pts[0]
                normal = np.cross(v1, v2)
                normal = normal / np.linalg.norm(normal)
                target_area = target_areas[i]
                joint_face = rwj.JointFace(normal, corner_pts, target_area)
                joint = rwj.Joint([joint_face])
                joint_group.append(joint)
            joint_group = rwj.JointGroup(joint_group)
            joint_group.set_degree_of_freedom(np.array([dof_lists[i].X, dof_lists[i].Y, dof_lists[i].Z]))
            joint_groups.append(joint_group)
        
        return [joint_groups]