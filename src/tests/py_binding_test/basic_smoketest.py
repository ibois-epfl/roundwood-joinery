import roundwoodJoineryBindings as rwj
import numpy as np
import Rhino

print(rwj.PI)

joints = []

for crv in i_crvs:
    corner_pts = [np.array([pt.X, pt.Y, pt.Z]) for pt in crv]
    normal = np.array([0, 0, 1])
    target_area = 12000.0 # mm^2
    joint_face = rwj.JointFace(normal, corner_pts, target_area)
    joint = rwj.Joint([joint_face])
    joints.append(joint)

beam_point_cloud = rwj.PointCloud(np.array([[pt.X, pt.Y, pt.Z] for pt in i_pointcloud.GetPoints()]))
beam = rwj.Beam(100.0, joints, [np.array([0, 0, 0])], beam_point_cloud)
print("Beam created with reference diameter:", beam.GetReferenceDiameter())