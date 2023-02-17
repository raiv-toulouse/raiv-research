#!/usr/bin/env python
# coding: utf-8

import rospy
from raiv_camera_calibration.perspective_calibration import PerspectiveCalibration
from raiv_libraries.robotUR import RobotUR
from raiv_libraries.robot_with_vaccum_gripper import Robot_with_vaccum_gripper
from raiv_research.srv import GetBestPrediction, ProcessNewImage
import geometry_msgs.msg as geometry_msgs
from raiv_libraries.image_tools import ImageTools
from raiv_libraries.get_coord_node import InBoxCoord
from raiv_libraries.srv import get_coordservice, PickingBoxIsEmpty, GetPickingBoxCentroid
from raiv_libraries.srv import ClearPrediction
from raiv_libraries import tools

Z_PICK_ROBOT = 0.12  # Z coord before going down to pick
X_OUT = 0.21  # XYZ coord where the robot is out of camera scope
Y_OUT = -0.27
Z_OUT = 0.16
X_PLACE = -0.003
Y_PLACE = -0.29
Z_PLACE = 0.16  # Z coord to start place movement (in meter)

# Declaration of services
#
# best_prediction_service
rospy.wait_for_service('/best_prediction_service')
best_prediction_service = rospy.ServiceProxy('/best_prediction_service', GetBestPrediction)
# Clear_Prediction
# GetNewImage
rospy.wait_for_service('/Process_new_images')
process_new_image_service = rospy.ServiceProxy('/Process_new_images', ProcessNewImage)
# In_box_coordService
rospy.wait_for_service('/In_box_coordService')
coord_service = rospy.ServiceProxy('/In_box_coordService', get_coordservice)
# Is_Picking_Box_Empty
rospy.wait_for_service('/Is_Picking_Box_Empty')
is_picking_box_empty_service = rospy.ServiceProxy('/Is_Picking_Box_Empty', PickingBoxIsEmpty)
# Get_picking_box_centroid
rospy.wait_for_service('/Get_picking_box_centroid')
get_picking_box_centroid_service = rospy.ServiceProxy('/Get_picking_box_centroid', GetPickingBoxCentroid)

if __name__ == "__main__":
    import argparse
    import time

    parser = argparse.ArgumentParser(description='Perform robot pick action at location received by best_prediction_service response')
    parser.add_argument('calibration_folder', type=str, help='calibration files folder')
    args = parser.parse_args()

    rospy.init_node("node_move_robot_to_prediction")
    persp_calib = PerspectiveCalibration(args.calibration_folder)
    robot = Robot_with_vaccum_gripper()
    picking_box_centroid = get_picking_box_centroid_service()

    # The robot must go out of the camera field
    robot.go_to_xyz_position(X_OUT, Y_OUT, Z_OUT, duration=2)
    process_new_image_service()  # Ask for a new image and start its processing (generation of predictions)
    while not is_picking_box_empty_service().empty_box:
        # Go to box centroid, during this time, predictions are processed
        coord_centroid = [picking_box_centroid.x_centroid, picking_box_centroid.y_centroid]
        x, y, z = persp_calib.from_2d_to_3d(coord_centroid)
        robot.go_to_xyz_position(x, y, Z_PICK_ROBOT, duration=2)
        resp = best_prediction_service()  # We can now ask a service to get the best prediction
        print('proba ---------------: ', resp.pred.proba)
        # Pick the piece
        coord_pixel = [resp.pred.x, resp.pred.y]
        x, y, z = persp_calib.from_2d_to_3d(coord_pixel)
        pose_for_pick = geometry_msgs.Pose(geometry_msgs.Vector3(x, y, Z_PICK_ROBOT), RobotUR.tool_down_pose)
        robot.pick(pose_for_pick)
        # Next, go to OUT position (out of camera scope)
        robot.go_to_xyz_position(X_OUT, Y_OUT, Z_OUT, duration=2)  # The robot must go out of the camera field
        process_new_image_service()  # Ask for a new image and start its processing (generation of predictions)
        if robot.check_if_object_gripped():  # An object is gripped
            # Place the object
            resp_place = coord_service('random_no_swap', InBoxCoord.PLACE, InBoxCoord.IN_THE_BOX, ImageTools.CROP_WIDTH, ImageTools.CROP_HEIGHT, None, None)
            place_pose = tools.xyz_to_pose(X_PLACE, Y_PLACE, Z_PLACE)
            robot.place(place_pose)
        else:  # Wait to let enough time for prediction computation
            rospy.sleep(2)
        robot.release_gripper()  # Switch off the gripper