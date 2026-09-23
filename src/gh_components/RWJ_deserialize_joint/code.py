"""This component deserializes the joint data from a file."""
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

class RWJ_deserialize_joinery(component):
    def RunScript(self, i_file_path: str) -> typing.List[rwj.JointGroup]:

        file_path = str(i_file_path)
        return rwj_serial.load_joinery_from_file(file_path)