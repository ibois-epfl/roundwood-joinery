"""Serialization / deserialization of joinery data (joint groups, joints, joint faces) to/from JSON."""

import json
import typing

import numpy as np

from . import roundwoodJoineryBindings as rwj


def _vec3_to_list(vector) -> typing.List[float]:
    return [float(vector[0]), float(vector[1]), float(vector[2])]


def _list_to_vec3(values: typing.List[float]) -> np.ndarray:
    return np.asarray(values, dtype=np.float64)


def _serialize_joint_face(face: rwj.JointFace) -> dict:
    return {
        "normal": _vec3_to_list(face.get_normal()),
        "corners": [_vec3_to_list(corner) for corner in face.get_corners()],
        "center": _vec3_to_list(face.get_center()),
        "target_area": face.get_target_area(),
        "current_area": face.get_current_area(),
        "max_allowable_depth": face.get_max_allowable_depth(),
        "projected_points": [_vec3_to_list(point) for point in face.get_projected_points()],
    }


def _deserialize_joint_face(data: dict) -> rwj.JointFace:
    # "center" and "current_area" are derived (center from corners at construction, current_area from
    # a point-cloud projection) and are not restored here: center is recomputed identically from the
    # corners, and current_area requires re-running ComputeCurrentAreaAndDepths against a point cloud.
    face = rwj.JointFace(
        _list_to_vec3(data["normal"]),
        [_list_to_vec3(corner) for corner in data["corners"]],
        data.get("target_area", 0.0),
        data.get("max_allowable_depth", 50.0),
    )
    face.set_projected_points([_list_to_vec3(point) for point in data.get("projected_points", [])])
    return face


def _serialize_joint(joint: rwj.Joint) -> dict:
    return {
        "center": _vec3_to_list(joint.get_center()),
        "closest_point_on_skeleton": _vec3_to_list(joint.get_closest_point_on_skeleton()),
        "remaining_area": joint.get_remaining_area(),
        "remaining_inertia": joint.get_remaining_inertia(),
        "remaining_section_outline": [_vec3_to_list(point) for point in joint.get_remaining_section_outline()],
        "initial_section_outline": [_vec3_to_list(point) for point in joint.get_initial_section_outline()],
        "faces": [_serialize_joint_face(face) for face in joint.get_faces()],
    }


def _deserialize_joint(data: dict) -> rwj.Joint:
    # "center" is not restored via a setter: Joint's constructor deterministically recomputes it as the
    # average of its faces' centers, which will match as long as the faces round-trip identically.
    faces = [_deserialize_joint_face(face) for face in data["faces"]]
    joint = rwj.Joint(faces)
    joint.set_closest_point_on_skeleton(_list_to_vec3(data["closest_point_on_skeleton"]))
    joint.set_remaining_area(data.get("remaining_area", 0.0))
    joint.set_remaining_inertia(data.get("remaining_inertia", 0.0))
    joint.set_remaining_section_outline(
        [_list_to_vec3(point) for point in data.get("remaining_section_outline", [])]
    )
    joint.set_initial_section_outline(
        [_list_to_vec3(point) for point in data.get("initial_section_outline", [])]
    )
    return joint


def _serialize_joint_group(joint_group: rwj.JointGroup) -> dict:
    joints = joint_group.get_joints()
    return {
        "degree_of_freedom": _vec3_to_list(joint_group.get_degree_of_freedom()),
        "centroid": _vec3_to_list(joint_group.get_centroid()) if joints else None,
        "joints": [_serialize_joint(joint) for joint in joints],
    }


def _deserialize_joint_group(data: dict) -> rwj.JointGroup:
    # "centroid" is not restored via a setter: it is always recomputed from the group's joints.
    joints = [_deserialize_joint(joint) for joint in data["joints"]]
    joint_group = rwj.JointGroup(joints)
    joint_group.set_degree_of_freedom(_list_to_vec3(data["degree_of_freedom"]))
    return joint_group


def serialize_joinery(joint_groups: typing.List[rwj.JointGroup]) -> dict:
    """
    Serializes all joint groups of a beam's joinery, and the joints/joint faces they contain, into a
    JSON-compatible dict.
    """
    return {"joint_groups": [_serialize_joint_group(joint_group) for joint_group in joint_groups]}


def deserialize_joinery(data: dict) -> typing.List[rwj.JointGroup]:
    """Reconstructs a list of JointGroup objects from a dict produced by serialize_joinery."""
    return [_deserialize_joint_group(joint_group) for joint_group in data["joint_groups"]]


def save_joinery_to_file(joint_groups: typing.List[rwj.JointGroup], filepath: str) -> None:
    with open(filepath, "w") as f:
        json.dump(serialize_joinery(joint_groups), f, indent=2)


def load_joinery_from_file(filepath: str) -> typing.List[rwj.JointGroup]:
    with open(filepath, "r") as f:
        return deserialize_joinery(json.load(f))
