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

class RWJ_transform_joinery(component):
    def RunScript(self,
            i_transformation: Rhino.Geometry.Transform,
            i_joinery: System.Collections.Generic.List[object]) -> typing.List[rwj.JointGroup]:
        o_joint_outlines = []
        o_joinery = []
        for joint_group in i_joinery:
            duplicated_joint_group = joint_group.duplicate()
            transformation = np.reshape(np.array(i_transformation.ToDoubleArray(True)), (4, 4))
            duplicated_joint_group.apply_transformation(transformation)
            for joint in duplicated_joint_group.get_joints():
                for face in joint.get_faces():
                    face_outline = Rhino.Geometry.Polyline(
                        [Rhino.Geometry.Point3d(pt[0], pt[1], pt[2]) for pt in face.get_corners()]
                        )
                    o_joint_outlines.append(face_outline)
            o_joinery.append(duplicated_joint_group)
        
        return [o_joinery, o_joint_outlines]