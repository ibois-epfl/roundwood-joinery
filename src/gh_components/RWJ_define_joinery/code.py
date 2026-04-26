"""This component defines the joinery for roundwood elements with its surfaces and degrees of freedom."""
#! python3

import System
import typing

import Rhino
import Grasshopper

import ghpythonlib.treehelpers as th
from ghpythonlib.componentbase import executingcomponent as component

import roundwood_joinery
import roundwood_joinery.roundwoodJoineryBindings as rwj

import numpy as np

class RWJ_define_joinery(component):
    def RunScript(self,
            i_curves_datatree: Grasshopper.DataTree[Rhino.Geometry.Polyline],
            i_dof_vectors: Grasshopper.DataTree[Rhino.Geometry.Vector3d],
            i_max_joint_depths_datatree: Grasshopper.DataTree[float],
            i_target_areas_datatree: Grasshopper.DataTree[float],
            i_jointface_normals: Grasshopper.DataTree[Rhino.Geometry.Vector3d]) -> typing.List[rwj.JointGroup]:
        curve_lists = th.tree_to_list(i_curves_datatree)
        if i_max_joint_depths_datatree.BranchCount > 0:
            max_joint_depths = th.tree_to_list(i_max_joint_depths_datatree)
        else:
            max_joint_depths = None
        target_areas = th.tree_to_list(i_target_areas_datatree)
        jointface_normals = th.tree_to_list(i_jointface_normals)
        print(target_areas)
        dof_lists = th.tree_to_list(i_dof_vectors)
        joint_groups = []
        for i, crv_group in enumerate(curve_lists):
            joint_group = []
            target_area = target_areas[i][0]
            if max_joint_depths is not None:
                max_joint_depth = max_joint_depths[i][0]
            else:
                max_joint_depth = None
            for  j, crv in enumerate(crv_group):
                corner_pts = [np.array([pt.X, pt.Y, pt.Z]) for pt in crv]
                if jointface_normals is None:
                    v1 = corner_pts[1] - corner_pts[0]
                    v2 = corner_pts[2] - corner_pts[0]
                    normal = np.cross(v1, v2)
                    normal = normal / np.linalg.norm(normal)
                else:
                    normal = np.array([jointface_normals[i][j].X, jointface_normals[i][j].Y, jointface_normals[i][j].Z])
                print(normal)
                if max_joint_depth is not None:
                    joint_face = rwj.JointFace(normal, corner_pts, target_area, max_joint_depth)
                else:
                    joint_face = rwj.JointFace(normal, corner_pts, target_area)
                joint = rwj.Joint([joint_face])
                joint_group.append(joint)
            joint_group = rwj.JointGroup(joint_group)
            joint_group.set_degree_of_freedom(np.array([dof_lists[i].X, dof_lists[i].Y, dof_lists[i].Z]))
            joint_groups.append(joint_group)
        
        return [joint_groups]