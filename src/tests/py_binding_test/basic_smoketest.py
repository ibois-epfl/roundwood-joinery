import roundwoodJoineryBindings as rwj
import numpy as np
import Rhino

import ghpythonlib.treehelpers as th

joints = []
closest_points_on_skeleton = []
i_crvs = th.tree_to_list(i_crvs)

pts = []

joint_groups = []
for i, crv_group in enumerate(i_crvs):
    joint_group = []
    for crv in crv_group:
        corner_pts = [np.array([pt.X, pt.Y, pt.Z]) for pt in crv]
        v1 = corner_pts[1] - corner_pts[0]
        v2 = corner_pts[2] - corner_pts[0]

        normal = np.array([i_normals[i].X, i_normals[i].Y, i_normals[i].Z])
        target_area = i_target_surfaces[i]
        joint_face = rwj.JointFace(normal, corner_pts, target_area)
        joint = rwj.Joint([joint_face])
        joint_group.append(joint)
    joint_group = rwj.JointGroup(joint_group)
    joint_group.set_degree_of_freedom(np.array([i_normals[i].X, i_normals[i].Y, i_normals[i].Z]))
    joint_groups.append(joint_group)

beam_point_cloud = rwj.PointCloud(np.array([[pt.X, pt.Y, pt.Z] for pt in i_pointcloud.GetPoints()]))
pc_skeleton = rwj.Utils.compute_point_cloud_skeleton(beam_point_cloud, 100.0, .01)
rh_pc_skeleton = Rhino.Geometry.PointCloud([Rhino.Geometry.Point3d(pt[0], pt[1], pt[2]) for pt in pc_skeleton])
beam = rwj.Beam(200.0, joint_groups, pc_skeleton, beam_point_cloud)
for joint_group in beam.get_joints_by_group():
    pt_on_skeleton = joint_group.get_joints()[0].get_closest_point_on_skeleton()
    closest_points_on_skeleton.append(Rhino.Geometry.Point3d(pt_on_skeleton[0], pt_on_skeleton[1], pt_on_skeleton[2]))
    for joints in joint_group.get_joints():
        for face in joints.get_faces():
            print(f"face normal: {face.get_normal()}")
            print(f"face current area: {face.compute_current_area(beam_point_cloud)}")
            print(f"face target area: {face.get_target_area()}")
            for point in face.project_points_onto_face(beam_point_cloud):
                pts.append(Rhino.Geometry.Point3d(point[0], point[1], point[2]))
transform_matrices = beam.compute_joint_group_optimisation(3, 1.0)

rh_transform_matrices = []
for transform in transform_matrices:
    rh_transform = Rhino.Geometry.Transform()
    for i in range(4):
        for j in range(4):
            rh_transform[i, j] = transform[i, j]
    rh_transform_matrices.append(rh_transform)