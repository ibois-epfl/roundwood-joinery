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
        for joint_group in i_joinery:

            transformation = np.reshape(np.array(i_transformation.ToDoubleArray(True)), (4, 4))
            joint_group.apply_transformation(transformation)
        
        return i_joinery