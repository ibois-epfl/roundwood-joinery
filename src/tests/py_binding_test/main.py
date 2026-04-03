from compas.colors import Color
from compas.geometry import Pointcloud
import compas.geometry as cg
from compas_viewer import Viewer

import numpy as np

import roundwoodJoineryBindings as rwj

def main():
    viewer = Viewer()
    pointcloud = Pointcloud.from_ply("../../../test_files/ply/cleaned_trunc_00094_subsampled.ply")
    viewer.scene.add(pointcloud, pointsize=2)

    corner1 = [680.0, -490.0, 420.0]
    corner2 = [680.0, -290.0, 420.0]
    corner3 = [750.0, -290.0, 350.0]
    corner4 = [750.0, -490.0, 350.0]
    corners = [corner1, corner2, corner3, corner4]
    np_corners = np.array(corners)
    normal = np.cross(np_corners[1] - np_corners[0], np_corners[2] - np_corners[0])
    target_area = 12000.0 # mm^2
    joint_face = rwj.JointFace(normal, np_corners, target_area)
    joint = rwj.Joint([joint_face])
    beam_point_cloud = rwj.PointCloud(np.array([[pt.x, pt.y, pt.z] for pt in pointcloud.points]))
    beam_skeleton = rwj.Utils.ComputePointCloudSkeleton(pointCloud=beam_point_cloud, alpha=300.0, offset=0.1)
    print(f"current face area: {joint_face.ComputeCurrentArea(beam_point_cloud)}")
    joint_polygon = cg.Polygon(corners)
    beam = rwj.Beam(200.0, [joint], beam_skeleton, beam_point_cloud)
    beam.FindJointClosestPointsOnSkeleton()
    res = beam.ComputeOneIterationOfJointFaceTranslationsForOptimisation()
    transform = rwj.Utils.ComputeApproximatingTransformation(res)
    print("transform as list:", transform.tolist())
    viewer.scene.add(joint_polygon, facecolor=Color.red(), edgecolor=Color.black(), linewidth=2)
    new_joint_polygon = cg.Polygon(corners)
    new_joint_polygon.transform(transform.tolist())
    viewer.scene.add(new_joint_polygon, facecolor=Color.green(), edgecolor=Color.black(), linewidth=2)

    viewer.show()


if __name__ == "__main__":
    main()
