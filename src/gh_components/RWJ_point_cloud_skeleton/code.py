#! python3

import System
import typing

import Rhino
import Grasshopper

from ghpythonlib.componentbase import executingcomponent as component

import roundwood_joinery
import roundwood_joinery.roundwoodJoineryBindings as rwj

import numpy as np

class RWJ_compute_point_cloud_skeleton(component):
    def RunScript(self, i_point_cloud: Rhino.Geometry.PointCloud, i_alpha, i_offset):
        if not i_alpha:
            i_alpha = 500
        if not i_offset:
            i_offset = 1
        np_point_cloud = np.array([[pt.X, pt.Y, pt.Z] for pt in i_point_cloud])
        rwj_pc = rwj.PointCloud(np_point_cloud)
        np_skeleton_pts = rwj.Utils.compute_point_cloud_skeleton(rwj_pc, alpha=i_alpha, offset=i_offset)
        
        o_skeleton = [Rhino.Geometry.Point3d(pt[0], pt[1], pt[2]) for pt in np_skeleton_pts]
        return [o_skeleton]