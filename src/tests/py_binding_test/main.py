from compas.colors import Color
from compas.geometry import Pointcloud
import compas.geometry as cg
from compas_viewer import Viewer

import numpy as np

import roundwoodJoineryBindings as rwj

def main():
    viewer = Viewer()
    pointcloud = Pointcloud.from_ply("../../../test_files/ply/cleaned_trunc_00094_subsampled.ply")
    viewer.scene.add(pointcloud, pointsize=3.0)

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
    grouped_joints = rwj.JointGroup([joint])

    beam_point_cloud = rwj.PointCloud(np.array([[pt.x, pt.y, pt.z] for pt in pointcloud.points]))
    beam_skeleton = rwj.Utils.compute_point_cloud_skeleton(pointCloud=beam_point_cloud, alpha=80.0, offset=0.001)

    corners1.append(corners1[0])
    corners2.append(corners2[0])
    corners3.append(corners3[0])

    joint_face_1_polyline = cg.Polyline(corners1)
    joint_face_2_polyline = cg.Polyline(corners2)
    joint_face_3_polyline = cg.Polyline(corners3)

    beam = rwj.Beam(250.0, [grouped_joints], beam_skeleton, beam_point_cloud)
    beam.find_joint_closest_points_on_skeleton()
    for joint_group in beam.get_joints_by_group():
        for joint in joint_group.get_joints():
            for i, joint_face in enumerate(joint.get_faces()):
                joint_actual_face = joint_face.get_current_outline(beam_point_cloud, 180.0)
                viewer.scene.add(cg.Polygon(joint_actual_face), facecolor=Color.red(), edgecolor=Color.black(), linewidth=2)
                print(f"Joint face {i+1} initial area:", joint_face.compute_current_area_and_depths(beam_point_cloud, 180.0)[0])

    skeleton_polyline = cg.Polyline(beam_skeleton)
    viewer.scene.add(skeleton_polyline, color=Color.blue(), linewidth=10)

    alpha = 5000.0
    beam.compute_joint_group_optimisation(500, 0.01, alpha, "test.csv")
    
    

    print(f"Joint face 1 current area after 1-step correction:, {beam.get_joints_by_group()[0].get_joints()[0].get_faces()[0].compute_current_area_and_depths(beam_point_cloud, 200, 500, 50)[0]:.2f}")
    print(f"Joint face 2 current area after 1-step correction:, {beam.get_joints_by_group()[0].get_joints()[0].get_faces()[1].compute_current_area_and_depths(beam_point_cloud, 200, 500, 50)[0]:.2f}")
    print(f"Joint face 3 current area after 1-step correction:, {beam.get_joints_by_group()[0].get_joints()[0].get_faces()[2].compute_current_area_and_depths(beam_point_cloud, 200, 500, 50)[0]:.2f}")

    new_joint_face_1_outline = beam.get_joints_by_group()[0].get_joints()[0].get_faces()[0].get_current_outline(beam_point_cloud, 200, 500, 50)
    new_joint_face_2_outline = beam.get_joints_by_group()[0].get_joints()[0].get_faces()[1].get_current_outline(beam_point_cloud, 200, 500, 50)
    new_joint_face_3_outline = beam.get_joints_by_group()[0].get_joints()[0].get_faces()[2].get_current_outline(beam_point_cloud, 200, 500, 50)

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
