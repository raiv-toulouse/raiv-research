#!/usr/bin/env python
# coding: utf-8

import rospy
from raiv_camera_calibration.perspective_calibration import PerspectiveCalibration
from raiv_libraries.robotUR import RobotUR
from raiv_libraries.robot_with_vaccum_gripper import Robot_with_vaccum_gripper
from raiv_research.srv import GetBestPrediction
import geometry_msgs.msg as geometry_msgs
from raiv_libraries.image_tools import ImageTools
from raiv_libraries.get_coord_node import InBoxCoord
from raiv_libraries.srv import get_coordservice
from raiv_libraries.srv import ClearPrediction, ClearPredictionResponse



Z_PICK_ROBOT = 0.12 # Z coord before going down to pick
X_OUT = 0.21  # XYZ coord where the robot is out of camera scope
Y_OUT = -0.27
Z_OUT = 0.16
X_PLACE = -0.003
Y_PLACE = -0.29
Z_PLACE = 0.16  # Z coord to start place movement (in meter)

rospy.wait_for_service('best_prediction_service')
call_service = rospy.ServiceProxy('best_prediction_service', GetBestPrediction)
rospy.wait_for_service('/Clear_Prediction')
call_clear = rospy.ServiceProxy('/Clear_Prediction', ClearPrediction)

def xyz_to_pose(x, y, z):
    return geometry_msgs.Pose(geometry_msgs.Vector3(x, y, z), RobotUR.tool_down_pose)

if __name__=="__main__":
    import argparse

    parser = argparse.ArgumentParser(description='Perform robot pick action at location received by best_prediction_service response')
    parser.add_argument('calibration_folder', type=str, help='calibration files folder')
    args = parser.parse_args()

    rospy.init_node("node_move_robot_to_prediction")
    dPoint = PerspectiveCalibration(args.calibration_folder)
    robot = Robot_with_vaccum_gripper()
    robot.go_to_xyz_position(X_OUT, Y_OUT, Z_OUT)  # Move the robot out of camera scope


    # We can now ask a service to get and process 3D images
    coord_service_name = 'In_box_coordService'
    rospy.wait_for_service(coord_service_name)
    coord_service = rospy.ServiceProxy(coord_service_name, get_coordservice)

    resp = call_service('classic', 'no manual')  # Pixel coord of best prediction without precision information
    print('proba ---------------: ', resp.pred.proba)
    coord_pixel = [resp.pred.x, resp.pred.y]
    x, y, z = dPoint.from_2d_to_3d(coord_pixel)
    pose_for_pick = geometry_msgs.Pose(geometry_msgs.Vector3(x, y, Z_PICK_ROBOT), RobotUR.tool_down_pose)
    object_gripped = robot.pick(pose_for_pick)
    object_gripped = robot.check_if_object_gripped()


    while True:
        robot.go_to_xyz_position(X_OUT, Y_OUT, Z_OUT, duration=2)
        # refresh of depth image
        resp_place = coord_service('random_no_swap', InBoxCoord.PLACE, InBoxCoord.IN_THE_BOX, ImageTools.CROP_WIDTH, ImageTools.CROP_HEIGHT, None, None)
        rep = call_clear()  # Launch the clear service to clear all prediction


        if object_gripped:
            # Place the object
            place_pose = xyz_to_pose(X_PLACE, Y_PLACE, Z_PLACE)
            robot.place(place_pose)
            robot.release_gripper()  # Switch off the gripper

        else:
            robot.release_gripper()  # Switch off the gripper

        # The robot must go out of the camera field
        robot.go_to_xyz_position(X_OUT, Y_OUT, Z_OUT, duration=2)

        resp = call_service('classic', 'no manual')  # Pixel coord of best prediction without precision information
        print('proba ---------------: ', resp.pred.proba)
        coord_pixel = [resp.pred.x, resp.pred.y]
        x, y, z = dPoint.from_2d_to_3d(coord_pixel)
        pose_for_pick = geometry_msgs.Pose(geometry_msgs.Vector3(x, y, Z_PICK_ROBOT), RobotUR.tool_down_pose)
        object_gripped = robot.pick(pose_for_pick)
        object_gripped = robot.check_if_object_gripped()



