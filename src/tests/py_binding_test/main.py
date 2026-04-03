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

    corner11 = [680.0, -490.0, 420.0]
    corner12 = [680.0, -290.0, 420.0]
    corner13 = [750.0, -290.0, 350.0]
    corner14 = [750.0, -490.0, 350.0]

    corner21 = [750.0, -290.0, 350.0]
    corner22 = [750.0, -490.0, 350.0]
    corner23 = [820.0, -490.0, 350.0]
    corner24 = [820.0, -290.0, 350.0]

    corner31 = [820.0, -490.0, 350.0]
    corner32 = [820.0, -290.0, 350.0]
    corner33 = [890.0, -290.0, 420.0]
    corner34 = [890.0, -490.0, 420.0]

    corners1 = [corner11, corner12, corner13, corner14]
    corners2 = [corner21, corner22, corner23, corner24]
    corners3 = [corner31, corner32, corner33, corner34]

    np_corners1 = np.array(corners1)
    np_corners2 = np.array(corners2)
    np_corners3 = np.array(corners3)

    normal1 = -1 * np.cross(np_corners1[1] - np_corners1[0], np_corners1[2] - np_corners1[0])
    normal2 = np.cross(np_corners2[1] - np_corners2[0], np_corners2[2] - np_corners2[0])
    normal3 = -1 * np.cross(np_corners3[1] - np_corners3[0], np_corners3[2] - np_corners3[0])


    joint_face_1 = rwj.JointFace(normal1, np_corners1, 5000.0)
    joint_face_2 = rwj.JointFace(normal2, np_corners2, 9000.0)
    joint_face_3 = rwj.JointFace(normal3, np_corners3, 5000.0)

    joint = rwj.Joint([joint_face_1, joint_face_2, joint_face_3])

    beam_point_cloud = rwj.PointCloud(np.array([[pt.x, pt.y, pt.z] for pt in pointcloud.points]))
    beam_skeleton = rwj.Utils.compute_point_cloud_skeleton(pointCloud=beam_point_cloud, alpha=80.0, offset=0.001)

    corners1.append(corners1[0])
    corners2.append(corners2[0])
    corners3.append(corners3[0])

    joint_face_1_polyline = cg.Polyline(corners1)
    joint_face_2_polyline = cg.Polyline(corners2)
    joint_face_3_polyline = cg.Polyline(corners3)

    beam = rwj.Beam(200.0, [joint], beam_skeleton, beam_point_cloud)
    beam.find_joint_closest_points_on_skeleton()
    for joint in beam.get_joints():
        for i, joint_face in enumerate(joint.get_faces()):
            joint_actual_face = joint_face.get_current_outline(beam_point_cloud)
            viewer.scene.add(cg.Polygon(joint_actual_face), facecolor=Color.red(), edgecolor=Color.black(), linewidth=2)
            print(f"Joint face {i+1} initial area:", joint_face.compute_current_area(beam_point_cloud))

    skeleton_polyline = cg.Polyline(beam_skeleton)
    viewer.scene.add(skeleton_polyline, color=Color.blue(), linewidth=10)

    res = beam.compute_one_iteration_of_joint_face_translations_for_optimisation()
    transform = rwj.Utils.compute_approximating_transformation(res)

    new_np_corners1 = np.hstack((np_corners1, np.ones((np_corners1.shape[0], 1))))
    new_np_corners1 = (transform @ new_np_corners1.T).T[:, :3]
    new_normal1 = (transform[:3, :3] @ normal1).tolist()

    new_np_corners2 = np.array(corners2)
    new_np_corners2 = np.hstack((new_np_corners2, np.ones((new_np_corners2.shape[0], 1))))
    new_np_corners2 = (transform @ new_np_corners2.T).T[:, :3]
    new_normal2 = (transform[:3, :3] @ normal2).tolist()

    new_np_corners3 = np.array(corners3)
    new_np_corners3 = np.hstack((new_np_corners3, np.ones((new_np_corners3.shape[0], 1))))
    new_np_corners3 = (transform @ new_np_corners3.T).T[:, :3]
    new_normal3 = (transform[:3, :3] @ normal3).tolist()

    new_joint_face_1 = rwj.JointFace(np.asarray(new_normal1), new_np_corners1, 5000.0)
    new_joint_face_2 = rwj.JointFace(np.asarray(new_normal2), new_np_corners2, 9000.0)
    new_joint_face_3 = rwj.JointFace(np.asarray(new_normal3), new_np_corners3, 5000.0)

    print(f"Joint face 1 current area after 1-step correction:, {new_joint_face_1.compute_current_area(beam_point_cloud):.2f}")
    print(f"Joint face 2 current area after 1-step correction:, {new_joint_face_2.compute_current_area(beam_point_cloud):.2f}")
    print(f"Joint face 3 current area after 1-step correction:, {new_joint_face_3.compute_current_area(beam_point_cloud):.2f}")

    new_joint_face_1_outline = new_joint_face_1.get_current_outline(beam_point_cloud)
    new_joint_face_2_outline = new_joint_face_2.get_current_outline(beam_point_cloud)
    new_joint_face_3_outline = new_joint_face_3.get_current_outline(beam_point_cloud)

    new_joint_face_1_polygon = cg.Polygon(new_joint_face_1_outline)
    new_joint_face_2_polygon = cg.Polygon(new_joint_face_2_outline)
    new_joint_face_3_polygon = cg.Polygon(new_joint_face_3_outline)

    viewer.scene.add(new_joint_face_1_polygon, facecolor=Color.green(), edgecolor=Color.black(), linewidth=2)
    viewer.scene.add(new_joint_face_2_polygon, facecolor=Color.green(), edgecolor=Color.black(), linewidth=2)
    viewer.scene.add(new_joint_face_3_polygon, facecolor=Color.green(), edgecolor=Color.black(), linewidth=2)

    viewer.scene.add(joint_face_1_polyline, facecolor=Color.red(), edgecolor=Color.red(), linewidth=2)
    viewer.scene.add(joint_face_2_polyline, facecolor=Color.red(), edgecolor=Color.red(), linewidth=2)
    viewer.scene.add(joint_face_3_polyline, facecolor=Color.red(), edgecolor=Color.red(), linewidth=2)

    viewer.show()


if __name__ == "__main__":
    main()
