gripper_open = 0.0
gripper_closed = 0.01

def plan_poses(node, coords, quats, grips):

    node.send_gripper_command(
        position=gripper_open,
        max_velocity=0.05,
    )

    prev_grip = 0
    for i in range(len(coords)):
        arm_traj = node.plan_arm_to_pose_constraints(
        group_name="arm",
        link_name="gripper_tcp",
        frame_id="world",
        goal_xyz=coords[i],
        goal_quat_wxyz=quats[i],
        )
        if arm_traj is not None:
            node.execute_moveit_trajectory(arm_traj)
        
        if not (grips[i] == prev_grip):
            if grips[i] == 1:
                node.send_gripper_command(
                position=gripper_closed,
                max_velocity=0.05,
                )
            else:
                node.send_gripper_command(
                position=gripper_open,
                max_velocity=0.05,
                )
            if i == len(coords)-1:
                arm_traj = node.plan_arm_to_pose_constraints(
                group_name="arm",
                link_name="gripper_tcp",
                frame_id="world",
                goal_xyz=coords[i],
                goal_quat_wxyz=quats[i],
                )
                if arm_traj is not None:
                    node.execute_moveit_trajectory(arm_traj)

        prev_grip = grips[i]

def pick_and_place(node, link, coord_0, quat_0, coord_1, quat_1):
    safe_height = 0.35
    low_height = 0.1
    node.send_gripper_command(
        position=gripper_open,
        max_velocity=0.05,
    )

    arm_traj = node.plan_arm_to_pose_constraints(
        group_name="arm",
        link_name=link,
        frame_id="world",
        goal_xyz=(coord_0[0], coord_0[1], safe_height),
        goal_quat_wxyz=quat_0,
        max_velocity_scaling = 2
        )
    if arm_traj is not None:
        print(arm_traj)
        node.execute_moveit_trajectory(arm_traj)

    arm_traj = node.plan_arm_to_pose_constraints(
        group_name="arm",
        link_name=link,
        frame_id="world",
        goal_xyz=(coord_0[0], coord_0[1], low_height),
        goal_quat_wxyz=quat_0,
        max_velocity_scaling = 0.5
        )
    if arm_traj is not None:
        node.execute_moveit_trajectory(arm_traj)

    arm_traj = node.plan_arm_to_pose_constraints(
        group_name="arm",
        link_name=link,
        frame_id="world",
        goal_xyz=coord_0,
        goal_quat_wxyz=quat_0,
        )
    if arm_traj is not None:
        node.execute_moveit_trajectory(arm_traj)

    node.send_gripper_command(
        position=gripper_closed,
        max_velocity=0.05
    )

    # arm_traj = node.plan_arm_to_pose_constraints(
    #     group_name="arm",
    #     link_name="gripper_tcp",
    #     frame_id="world",
    #     goal_xyz=coord_0,
    #     goal_quat_wxyz=quat_0,
    #     )
    # if arm_traj is not None:
    #     node.execute_moveit_trajectory(arm_traj)

    arm_traj = node.plan_arm_to_pose_constraints(
        group_name="arm",
        link_name=link,
        frame_id="world",
        goal_xyz=(coord_0[0], coord_0[1], low_height),
        goal_quat_wxyz=quat_0,

        )
    if arm_traj is not None:
        node.execute_moveit_trajectory(arm_traj)

    arm_traj = node.plan_arm_to_pose_constraints(
        group_name="arm",
        link_name=link,
        frame_id="world",
        goal_xyz=(coord_0[0], coord_0[1], safe_height),
        goal_quat_wxyz=quat_0,
        max_velocity_scaling = 0.5
        )
    if arm_traj is not None:
        node.execute_moveit_trajectory(arm_traj)

    arm_traj = node.plan_arm_to_pose_constraints(
        group_name="arm",
        link_name=link,
        frame_id="world",
        goal_xyz=(coord_1[0], coord_1[1], safe_height),
        goal_quat_wxyz=quat_1,
        max_velocity_scaling = 0.5
        )
    if arm_traj is not None:
        node.execute_moveit_trajectory(arm_traj)

    arm_traj = node.plan_arm_to_pose_constraints(
        group_name="arm",
        link_name=link,
        frame_id="world",
        goal_xyz=(coord_1[0], coord_1[1], low_height),
        goal_quat_wxyz=quat_1,
        max_velocity_scaling = 0.5
        )
    if arm_traj is not None:
        node.execute_moveit_trajectory(arm_traj)
    
    arm_traj = node.plan_arm_to_pose_constraints(
        group_name="arm",
        link_name=link,
        frame_id="world",
        goal_xyz=coord_1,
        goal_quat_wxyz=quat_1,
        )
    if arm_traj is not None:
        node.execute_moveit_trajectory(arm_traj)

    node.send_gripper_command(
        position=gripper_open,
        max_velocity=0.05,
    )

    arm_traj = node.plan_arm_to_pose_constraints(
        group_name="arm",
        link_name=link,
        frame_id="world",
        goal_xyz=(coord_1[0], coord_1[1], low_height),
        goal_quat_wxyz=quat_1,
        )
    if arm_traj is not None:
        node.execute_moveit_trajectory(arm_traj)

    arm_traj = node.plan_arm_to_pose_constraints(
        group_name="arm",
        link_name=link,
        frame_id="world",
        goal_xyz=(coord_1[0], coord_1[1], safe_height),
        goal_quat_wxyz=quat_1,
        max_velocity_scaling = 0.5
        )
    if arm_traj is not None:
        node.execute_moveit_trajectory(arm_traj)
    


