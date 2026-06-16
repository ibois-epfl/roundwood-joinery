"""This component aligns a set of points to a beam skeleton."""
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
            i_points: System.Collections.Generic.List[Rhino.Geometry.Point3d],
            i_skeleton: System.Collections.Generic.List[Rhino.Geometry.Point3d]) -> typing.List[Rhino.Geometry.Transform]:
        if i_points is None or i_skeleton is None:
            return None

        points_array = np.array([[pt.X, pt.Y, pt.Z] for pt in i_points])
        skeleton_array = np.array([[pt.X, pt.Y, pt.Z] for pt in i_skeleton])

        transformation = rwj.Utils.compute_curve_to_curve_transformation(skeleton_array, points_array)
        rh_transform = Rhino.Geometry.Transform()

        for i in range(4):
            for j in range(4):
                rh_transform[i, j] = transformation[i, j]
        return rh_transform