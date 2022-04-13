#!/usr/bin/env python
# coding: utf-8

import rospy
from raiv_camera_calibration.perspective_calibration import PerspectiveCalibration
from raiv_libraries.robotUR import RobotUR
from raiv_libraries.robot_with_vaccum_gripper import Robot_with_vaccum_gripper
from raiv_research.srv import GetBestPrediction
import geometry_msgs.msg as geometry_msgs


Z_PICK_ROBOT = 0.15 # Z coord before going down to pick
X_OUT = 0.0  # XYZ coord where the robot is out of camera scope
Y_OUT = -0.3
Z_OUT = 0.1


def get_best_prediction_coord():
    """ With best_prediction_service, ask for the best prediction provided by node_best_prediction.py """
    rospy.wait_for_service('best_prediction_service')
    try:
        service_add = rospy.ServiceProxy('best_prediction_service', GetBestPrediction)
        resp = service_add()
        coord = [resp.pred.x, resp.pred.y]
        return coord
    except rospy.ServiceException as e:
        print("Service call failes: %s"%e)


if __name__=="__main__":
    import argparse

    parser = argparse.ArgumentParser(description='Perform robot pick action at location received by best_prediction_service response')
    parser.add_argument('calibration_folder', type=str, help='calibration files folder')
    args = parser.parse_args()

    rospy.init_node("node_move_robot_to_prediction")
    dPoint = PerspectiveCalibration(args.calibration_folder)
    robot = Robot_with_vaccum_gripper()
    robot.go_to_xyz_position(X_OUT, Y_OUT, Z_OUT)  # Move the robot out of camera scope

    while True:
        coord_pixel = get_best_prediction_coord() # Pixel coord of best prediction
        x, y, z = dPoint.from_2d_to_3d(coord_pixel)
        pose_for_pick = geometry_msgs.Pose(
            geometry_msgs.Vector3(x, y, Z_PICK_ROBOT), RobotUR.tool_down_pose
        )
        robot.pick(pose_for_pick)
        # The robot must go out of the camera field
        robot.go_to_xyz_position(X_OUT, Y_OUT, Z_OUT)

