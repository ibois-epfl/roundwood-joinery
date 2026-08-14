"""This component optimizes the placement of joints for roundwood elements."""
#! python3

import System
import typing
import datetime

import Rhino
import Grasshopper

from ghpythonlib.componentbase import executingcomponent as component

import roundwood_joinery.serialization as rwj_serial
import roundwood_joinery.roundwoodJoineryBindings as rwj

import numpy as np

class RWJ_serialize_joinery(component):
    def RunScript(self,
            i_joinery: System.Collections.Generic.List[object],
            i_file_path: str):

        file_path = str(i_file_path) + datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S") + ".json"
        rwj_serial.save_joinery_to_file(i_joinery, file_path)