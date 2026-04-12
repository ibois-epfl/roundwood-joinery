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
    def RunScript(self, i_beam):
        print("hello")
        o_joint_areas = []
        o_joint_points = []
        o_joint_face_outlines = []
        o_joint_outlines = []
        for joint_group in i_beam.get_joints_by_group():
            areas_per_joint = []
            points_per_joints = []
            outlines_per_joint = []
            for joint in joint_group.get_joints():
                areas_per_face =  []
                points_per_face = []
                outlines_per_face = []
                for face in joint.get_faces():
                    points_on_face = []
                    areas_per_face.append(face.get_current_area())
                    for point in face.project_points_onto_face(i_beam.get_point_cloud()):
                        points_on_face.append(Rhino.Geometry.Point3d(point[0],point[1], point[2]))
                    points_per_face.append(points_on_face)
                    outline_pts = [Rhino.Geometry.Point3d(pt[0], pt[1], pt[2]) for pt in face.get_current_outline(i_beam.get_point_cloud())]
                    outline_pts.append(Rhino.Geometry.Point3d(face.get_current_outline(i_beam.get_point_cloud())[0][0], 
                                                              face.get_current_outline(i_beam.get_point_cloud())[0][1], 
                                                              face.get_current_outline(i_beam.get_point_cloud())[0][2]))
                    outlines_per_face.append(Rhino.Geometry.Polyline(outline_pts))
                points_per_joints.append(points_per_face)
                areas_per_joint.append(areas_per_face)
                outlines_per_joint.append(outlines_per_face)
            o_joint_points.append(points_per_joints)
            o_joint_areas.append(areas_per_joint)
            o_joint_outlines.append(outlines_per_joint)
        o_joint_areas = th.list_to_tree(o_joint_areas)
        o_joint_points = th.list_to_tree(o_joint_points)
        o_joint_outlines = th.list_to_tree(o_joint_outlines)

        o_beam_skeleton = Rhino.Geometry.Polyline([Rhino.Geometry.Point3d(pt[0], pt[1], pt[2]) for pt in i_beam.get_skeleton()])

        return [o_joint_areas, o_joint_points, o_joint_outlines, o_beam_skeleton]