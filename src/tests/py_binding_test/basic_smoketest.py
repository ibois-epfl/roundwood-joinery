import roundwoodJoineryBindings as rwj
import numpy as np
import Rhino

print(rwj.M_PI)

for crv in i_crvs:
    corner_pts = [np.array([pt.X, pt.Y, pt.Z]) for pt in crv]
    normal = np.array([0, 0, 1])
    target_area = 12000.0 # mm^2
    joint_face = rwj.JointFace(normal, corner_pts, target_area)
    print(joint_face.GetNormal())