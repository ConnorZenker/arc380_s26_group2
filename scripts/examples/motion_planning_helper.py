from get_image import request_capture
import numpy as np
import cv2
from cv2 import aruco
import angle_convert

gripper_open = 0.0
gripper_closed = 0.01
safe_height = 0.35
low_height = 0.15

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
        max_velocity_scaling = 0.05
        )
    if arm_traj is not None:
        arm_traj.joint_trajectory.points = [arm_traj.joint_trajectory.points[i] for i in range(len(arm_traj.joint_trajectory.points)) if (i % 5) == 0]
        node.execute_moveit_trajectory(arm_traj)

    arm_traj = node.plan_arm_to_pose_constraints(
        group_name="arm",
        link_name=link,
        frame_id="world",
        goal_xyz=(coord_0[0], coord_0[1], low_height),
        goal_quat_wxyz=quat_0,
        max_velocity_scaling = 0.05
        )
    if arm_traj is not None:
        arm_traj.joint_trajectory.points = [arm_traj.joint_trajectory.points[i] for i in range(len(arm_traj.joint_trajectory.points)) if (i % 5) == 0]
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
        max_velocity_scaling = 0.05
        )
    if arm_traj is not None:
        arm_traj.joint_trajectory.points = [arm_traj.joint_trajectory.points[i] for i in range(len(arm_traj.joint_trajectory.points)) if (i % 5) == 0]
        node.execute_moveit_trajectory(arm_traj)

    arm_traj = node.plan_arm_to_pose_constraints(
        group_name="arm",
        link_name=link,
        frame_id="world",
        goal_xyz=(coord_1[0], coord_1[1], safe_height),
        goal_quat_wxyz=quat_1,
        max_velocity_scaling = 0.05
        )
    if arm_traj is not None:
        arm_traj.joint_trajectory.points = [arm_traj.joint_trajectory.points[i] for i in range(len(arm_traj.joint_trajectory.points)) if (i % 5) == 0]
        node.execute_moveit_trajectory(arm_traj)

    arm_traj = node.plan_arm_to_pose_constraints(
        group_name="arm",
        link_name=link,
        frame_id="world",
        goal_xyz=(coord_1[0], coord_1[1], low_height),
        goal_quat_wxyz=quat_1,
        max_velocity_scaling = 0.05
        )
    if arm_traj is not None:
        arm_traj.joint_trajectory.points = [arm_traj.joint_trajectory.points[i] for i in range(len(arm_traj.joint_trajectory.points)) if (i % 5) == 0]
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
        arm_traj.joint_trajectory.points = [arm_traj.joint_trajectory.points[i] for i in range(len(arm_traj.joint_trajectory.points)) if (i % 5) == 0]
        node.execute_moveit_trajectory(arm_traj)
    
def correct_position(node, link, coord, quat, onEdge):
    tool_coord = (-0.016, 0.551, 0.092)
    tool_quat = (0,1,0,0)
    offset_x = 0.005
    offset_y = 0.005

    if onEdge:
        width = 0.014
        height = 0.023/2
    else:
        width = 0.023
        height = 0.014/2
    
    if quat == (0,1,0,0):
        coord_0_offsets = [0,
                           0,
                           0,
                           0,
                           (0.051/2) + (0.023/2) + offset_y,
                           (0.051/2) + (0.023/2),
                           -(0.051/2) - (0.023/2) - offset_y,
                           -(0.051/2) - (0.023/2)]
        coord_1_offsets = [(width/2) + (0.023/2) + offset_x,
                           (width/2) + (0.023/2) - 0.001,
                           -(width/2) - (0.023/2)- offset_x,
                           -(width/2) - (0.023/2),
                           0,
                           0,
                           0,
                           0]
        quat_offsets = [(0,1,0,0),
                        (0,0,1,0),
                        (0,0.7071,-0.7071,0),
                        (0,0.7071,0.7071,0)]
    else:
        coord_1_offsets = [0,
                           0,
                           0,
                           0,
                           (0.051/2) + (0.023/2) + 0.002,
                           (0.051/2) + (0.023/2),
                           -(0.051/2) - (0.023/2) - 0.002,
                           -(0.051/2) - (0.023/2)]
        coord_0_offsets = [(width/2) + (0.023/2) + 0.005,
                           (width/2) + (0.023/2) - 0.001,
                           -(width/2) - (0.023/2)-0.005,
                           -(width/2) - (0.023/2),
                           0,
                           0,
                           0,
                           0]
        quat_offsets = [(0,0.7071,0.7071,0),
                        (0,0.7071,-0.7071,0),
                        (0,1,0,0),
                        (0,0,1,0)]

    node.send_gripper_command(
        position=gripper_open,
        max_velocity=0.05,
    )

    arm_traj = node.plan_arm_to_pose_constraints(
        group_name="arm",
        link_name=link,
        frame_id="world",
        goal_xyz=(tool_coord[0], tool_coord[1], safe_height),
        goal_quat_wxyz=tool_quat,
        max_velocity_scaling = 0.05
        )
    if arm_traj is not None:
        node.execute_moveit_trajectory(arm_traj)

    arm_traj = node.plan_arm_to_pose_constraints(
        group_name="arm",
        link_name=link,
        frame_id="world",
        goal_xyz=(tool_coord[0], tool_coord[1], low_height),
        goal_quat_wxyz=tool_quat,
        max_velocity_scaling = 0.05
        )
    if arm_traj is not None:
        node.execute_moveit_trajectory(arm_traj)

    arm_traj = node.plan_arm_to_pose_constraints(
        group_name="arm",
        link_name=link,
        frame_id="world",
        goal_xyz=tool_coord,
        goal_quat_wxyz=tool_quat,
        max_velocity_scaling = 0.01
        )
    if arm_traj is not None:
        node.execute_moveit_trajectory(arm_traj)

    node.send_gripper_command(
        position=gripper_closed,
        max_velocity=0.05,
    )

    arm_traj = node.plan_arm_to_pose_constraints(
        group_name="arm",
        link_name=link,
        frame_id="world",
        goal_xyz=(tool_coord[0], tool_coord[1], tool_coord[2] + 0.02),
        goal_quat_wxyz=tool_quat,
        max_velocity_scaling = 0.01
        )
    if arm_traj is not None:
        node.execute_moveit_trajectory(arm_traj)

    arm_traj = node.plan_arm_to_pose_constraints(
        group_name="arm",
        link_name=link,
        frame_id="world",
        goal_xyz=(tool_coord[0], tool_coord[1] - 0.04, tool_coord[2] + 0.02),
        goal_quat_wxyz=tool_quat,
        max_velocity_scaling = 0.01
        )
    if arm_traj is not None:
        node.execute_moveit_trajectory(arm_traj)

    arm_traj = node.plan_arm_to_pose_constraints(
        group_name="arm",
        link_name=link,
        frame_id="world",
        goal_xyz=(tool_coord[0], tool_coord[1] - 0.04, safe_height),
        goal_quat_wxyz=tool_quat,
        max_velocity_scaling = 0.01
        )
    if arm_traj is not None:
        node.execute_moveit_trajectory(arm_traj)

    arm_traj = node.plan_arm_to_pose_constraints(
        group_name="arm",
        link_name=link,
        frame_id="world",
        goal_xyz=(coord[0] + coord_0_offsets[0], coord[1] + coord_1_offsets[0], safe_height),
        goal_quat_wxyz=quat_offsets[0],
        max_velocity_scaling = 0.01
        )
    if arm_traj is not None:
        node.execute_moveit_trajectory(arm_traj)


    for i in range(4):
        arm_traj = node.plan_arm_to_pose_constraints(
        group_name="arm",
        link_name=link,
        frame_id="world",
        goal_xyz=(coord[0] + coord_0_offsets[2*i], coord[1] + coord_1_offsets[2*i], coord[2] - height + tool_coord[2] + 0.05),
        goal_quat_wxyz=quat_offsets[i],
        max_velocity_scaling = 0.01
        )
        if arm_traj is not None:
            node.execute_moveit_trajectory(arm_traj)
        
        arm_traj = node.plan_arm_to_pose_constraints(
        group_name="arm",
        link_name=link,
        frame_id="world",
        goal_xyz=(coord[0] + coord_0_offsets[2*i], coord[1] + coord_1_offsets[2*i], coord[2] - height + tool_coord[2] - 0.021 + 0.001),
        goal_quat_wxyz=quat_offsets[i],
        max_velocity_scaling = 0.01
        )
        if arm_traj is not None:
            node.execute_moveit_trajectory(arm_traj)
        
        arm_traj = node.plan_arm_to_pose_constraints(
        group_name="arm",
        link_name=link,
        frame_id="world",
        goal_xyz=(coord[0] + coord_0_offsets[2*i + 1], coord[1] + coord_1_offsets[2*i + 1], coord[2] - height + tool_coord[2] - 0.021 + 0.001),
        goal_quat_wxyz=quat_offsets[i],
        max_velocity_scaling = 0.01
        )
        if arm_traj is not None:
            node.execute_moveit_trajectory(arm_traj)
        
        arm_traj = node.plan_arm_to_pose_constraints(
        group_name="arm",
        link_name=link,
        frame_id="world",
        goal_xyz=(coord[0] + coord_0_offsets[2*i + 1], coord[1] + coord_1_offsets[2*i + 1], coord[2] - height + tool_coord[2] + 0.05),
        goal_quat_wxyz=quat_offsets[i],
        max_velocity_scaling = 0.01
        )
        if arm_traj is not None:
            node.execute_moveit_trajectory(arm_traj)

    arm_traj = node.plan_arm_to_pose_constraints(
        group_name="arm",
        link_name=link,
        frame_id="world",
        goal_xyz=(coord[0] + coord_0_offsets[7], coord[1] + coord_1_offsets[7], safe_height),
        goal_quat_wxyz=quat_offsets[3],
        max_velocity_scaling = 0.01
        )
    if arm_traj is not None:
        node.execute_moveit_trajectory(arm_traj)

    arm_traj = node.plan_arm_to_pose_constraints(
        group_name="arm",
        link_name=link,
        frame_id="world",
        goal_xyz=(tool_coord[0], tool_coord[1] - 0.04, safe_height),
        goal_quat_wxyz=tool_quat,
        max_velocity_scaling = 0.01
        )
    if arm_traj is not None:
        node.execute_moveit_trajectory(arm_traj)

    arm_traj = node.plan_arm_to_pose_constraints(
        group_name="arm",
        link_name=link,
        frame_id="world",
        goal_xyz=(tool_coord[0], tool_coord[1] - 0.04, tool_coord[2] + 0.02),
        goal_quat_wxyz=tool_quat,
        max_velocity_scaling = 0.01
        )
    if arm_traj is not None:
        node.execute_moveit_trajectory(arm_traj)
    
    arm_traj = node.plan_arm_to_pose_constraints(
        group_name="arm",
        link_name=link,
        frame_id="world",
        goal_xyz=(tool_coord[0], tool_coord[1], tool_coord[2] + 0.02),
        goal_quat_wxyz=tool_quat,
        max_velocity_scaling = 0.01
        )
    if arm_traj is not None:
        node.execute_moveit_trajectory(arm_traj)

    arm_traj = node.plan_arm_to_pose_constraints(
        group_name="arm",
        link_name=link,
        frame_id="world",
        goal_xyz=(tool_coord[0], tool_coord[1], tool_coord[2] + 0.01),
        goal_quat_wxyz=tool_quat,
        max_velocity_scaling = 0.01
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
        goal_xyz=(tool_coord[0], tool_coord[1], low_height),
        goal_quat_wxyz=tool_quat,
        max_velocity_scaling = 0.05
        )
    if arm_traj is not None:
        node.execute_moveit_trajectory(arm_traj)


    arm_traj = node.plan_arm_to_pose_constraints(
        group_name="arm",
        link_name=link,
        frame_id="world",
        goal_xyz=(tool_coord[0], tool_coord[1], safe_height),
        goal_quat_wxyz=tool_quat,
        max_velocity_scaling = 0.05
        )
    if arm_traj is not None:
        node.execute_moveit_trajectory(arm_traj)
    
def get_new_block(node, link, coord, quat):

    arm_traj = node.plan_arm_to_pose_constraints(
        group_name="arm",
        link_name=link,
        frame_id="world",
        goal_xyz=(0.38, 0, 0.4),
        goal_quat_wxyz=(0,1,0,0),
        max_velocity_scaling = 0.05
        )
    if arm_traj is not None:
        node.execute_moveit_trajectory(arm_traj)

    img, depth, meta = request_capture()
    # img_path = "C:\\Users\\arc380\\arc380_s26_group2\\realsense_shared\\color.png"
    # img = cv2.imread(img_path)
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    # Load the predefined dictionary where our markers are printed from
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)

    # Load the default detector parameters
    detector_params = aruco.DetectorParameters()

    # Create an ArucoDetector using the dictionary and detector parameters
    detector = aruco.ArucoDetector(dictionary, detector_params)

    corners, ids, rejected = detector.detectMarkers(img)

    width = 10      # inches
    height = 7.5    # inches
    ppi = 96        # pixels per inch (standard resolution for most screens - can be any arbitrary value that still preserves information)

    _ids = ids.flatten()

    # Sort corners based on id
    ids = ids.flatten()
    #print(ids)

    # Sort the corners based on the ids
    corners = np.array([corners[i] for i in np.argsort(ids)])
    #print(corners.shape)

    # Remove dimensions of size 1
    corners = np.squeeze(corners)
    #print(corners)

    # Sort the ids
    ids = np.sort(ids)

    # Extract source points corresponding to the exterior bounding box corners of the 4 markers
    src_pts = np.array([corners[0][0], corners[1][1], corners[2][2], corners[3][3]], dtype='float32')

    dst_pts = np.array([[0, 0], [0, height*ppi], [width*ppi, height*ppi], [width*ppi, 0]], dtype='float32')

    M = cv2.getPerspectiveTransform(src_pts, dst_pts)

    # Apply the perspective transformation to the input image
    corrected_img = cv2.warpPerspective(img, M, (img_rgb.shape[1], img_rgb.shape[0]))

    # Crop the output image to the specified dimensions
    corrected_img = corrected_img[:int(height*ppi), :int(width*ppi)]

    img_data = corrected_img.reshape((-1, 3))
    img_data = np.float32(img_data)

    # Define the number of clusters
    k = 3

    # Define the criteria for the k-means algorithm
    # This is a tuple with three elements: (type of termination criteria, maximum number of iterations, epsilon/required accuracy)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)

    # Run the k-means algorithm
    # Parameters: data, number of clusters, best labels, criteria, number of attempts, initial centers
    _, labels, centers = cv2.kmeans(img_data, k, None, criteria, 10, cv2.KMEANS_RANDOM_CENTERS)

    centers = np.uint8(centers)

    # Rebuild the image using the labels and centers
    kmeans_data = centers[labels.flatten()]
    kmeans_img = kmeans_data.reshape(corrected_img.shape)
    labels = labels.reshape(corrected_img.shape[:2])

    beige = np.array([90, 100, 120])
    distances = np.linalg.norm(centers - beige, axis=1)
    beige_cluster_label = np.argmin(distances)

    mask_img = np.zeros(kmeans_img.shape[:2], dtype='uint8')
    mask_img[labels == beige_cluster_label] = 255

    contours, _ = cv2.findContours(mask_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    areas = [cv2.contourArea(contour) for contour in contours]
    expected_area = 2.1 * 1.1 * ppi**2
    closest_area_idx = np.argmin(np.abs(np.array(areas) - expected_area))

    selected_contour = contours[closest_area_idx]
    moments = cv2.moments(selected_contour)
    u_c = int(moments['m10']/moments['m00'])
    v_c = int(moments['m01']/moments['m00'])

    x_in = u_c / ppi
    y_in = v_c / ppi
    x_mm = x_in * 25.4
    y_mm = y_in * 25.4
    x_m = x_mm / 1000
    y_m = y_mm / 1000

    # [-0.068,0.271,0.021]
    rect = cv2.minAreaRect(selected_contour)
    if rect[1][0] >= rect[1][1]:
        angle = rect[2]
    else:
        angle = 90+rect[2]

    print(angle)
    rot_mat = angle_convert.euler_angles_to_rotation_matrix([angle-90, 0, 180])
    block_quat = angle_convert.rotation_matrix_to_quaternion(rot_mat)
    block_coord = (-0.068-y_m, 0.271 + x_m , 0.028)
    print(block_coord)
    print(block_quat)

    pick_and_place(node, link, block_coord, block_quat, coord, quat)



