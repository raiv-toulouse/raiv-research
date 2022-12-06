    rospy.init_node("node_move_robot_to_prediction")
    dPoint = PerspectiveCalibration(args.calibration_folder)
    robot = Robot_with_vaccum_gripper()
    robot.go_to_xyz_position(X_OUT, Y_OUT, Z_OUT)  # Move the robot out of camera scope

    xc = 0.327
    yc = 0.098
    zc = 0.160

    while not is_picking_box_empty_service().empty_box:
        # We can now ask a service to get and process 3D images
        resp = best_prediction_service()
        print('proba ---------------: ', resp.pred.proba)
        coord_pixel = [resp.pred.x, resp.pred.y]
        x, y, z = dPoint.from_2d_to_3d(coord_pixel)
        pose_for_pick = geometry_msgs.Pose(geometry_msgs.Vector3(x, y, Z_PICK_ROBOT), RobotUR.tool_down_pose)
        robot.pick(pose_for_pick)
        robot.go_to_xyz_position(X_OUT, Y_OUT, Z_OUT, duration=2)
        print('pos repos')
        clear_prediction_service()  # Launch the clear service to clear all prediction
        print('clear + new image')
        if robot.check_if_object_gripped(): # An object is gripped
            print('Object gripped')
            # Place the object
            resp_place = coord_service('random_no_swap', InBoxCoord.PLACE, InBoxCoord.IN_THE_BOX, ImageTools.CROP_WIDTH, ImageTools.CROP_HEIGHT, None, None)
            place_pose = tools.xyz_to_pose(X_PLACE, Y_PLACE, Z_PLACE)
            robot.place(place_pose)
        robot.release_gripper()  # Switch off the gripper
        # The robot must go out of the camera field
        robot.go_to_xyz_position(X_OUT, Y_OUT, Z_OUT, duration=2)
