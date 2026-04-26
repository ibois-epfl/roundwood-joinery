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
    def RunScript(self, i_beam):
        o_text_dots = []
        o_joint_areas = []
        o_joint_points = []
        o_joint_face_outlines = []
        o_joint_outlines = []
        o_jointface_normals = []
        for joint_group in i_beam.get_joints_by_group():
            areas_per_joint = []
            points_per_joints = []
            outlines_per_joint = []
            jointface_normals_per_joint = []
            for joint in joint_group.get_joints():
                areas_per_face =  []
                points_per_face = []
                outlines_per_face = []
                jointface_normal_per_face = []
                for face in joint.get_faces():
                    points_on_face = []
                    current_area = face.compute_current_area_and_depth(i_beam.get_point_cloud())[0]
                    target_area = face.get_target_area()
                    area_ratio = current_area/target_area
                    face_center = face.get_center()
                    text_dot = Rhino.Geometry.TextDot(f"area ratio = {area_ratio:.3f}", Rhino.Geometry.Point3d(face_center[0], face_center[1], face_center[2]))
                    o_text_dots.append(text_dot)
                    areas_per_face.append(current_area)
                    for point in face.project_points_onto_face(i_beam.get_point_cloud()):
                        points_on_face.append(Rhino.Geometry.Point3d(point[0],point[1], point[2]))
                    points_per_face.append(points_on_face)
                    if len(face.get_current_outline(i_beam.get_point_cloud())) > 0:
                        outline_pts = [Rhino.Geometry.Point3d(pt[0], pt[1], pt[2]) for pt in face.get_current_outline(i_beam.get_point_cloud())]
                        outline_pts.append(Rhino.Geometry.Point3d(face.get_current_outline(i_beam.get_point_cloud())[0][0], 
                                                                face.get_current_outline(i_beam.get_point_cloud())[0][1], 
                                                                face.get_current_outline(i_beam.get_point_cloud())[0][2]))
                        outlines_per_face.append(Rhino.Geometry.Polyline(outline_pts))
                    jointface_normal_per_face.append(Rhino.Geometry.Vector3d(face.get_normal()[0], face.get_normal()[1], face.get_normal()[2]))
                points_per_joints.append(points_per_face)
                areas_per_joint.append(areas_per_face)
                outlines_per_joint.append(outlines_per_face)
                jointface_normals_per_joint.append(jointface_normal_per_face)
            o_joint_points.append(points_per_joints)
            o_joint_areas.append(areas_per_joint)
            o_joint_outlines.append(outlines_per_joint)
            o_jointface_normals.append(jointface_normals_per_joint)
        o_joint_areas = th.list_to_tree(o_joint_areas)
        o_joint_points = th.list_to_tree(o_joint_points)
        o_joint_outlines = th.list_to_tree(o_joint_outlines)
        o_jointface_normals = th.list_to_tree(o_jointface_normals)
        o_beam_skeleton = Rhino.Geometry.Polyline([Rhino.Geometry.Point3d(pt[0], pt[1], pt[2]) for pt in i_beam.get_skeleton()])

        return [o_joint_areas, o_joint_points, o_joint_outlines, o_jointface_normals, o_beam_skeleton, o_text_dots]