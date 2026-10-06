#!/usr/bin/env python
"""
Cleaned version of process_urdf.py
- Removed unused multi-sphere deformation / large IK solving block (not used in final JSON)
- Removed or made optional heavy visualization code
- Cleaner structure and comments
- Core pipeline preserved for generating sphere_cik.json
"""

import numpy as np
import json
import os
import tf.transformations as tf
import time
import argparse
from utils import *   # Consolidated utilities

def make_parser():
    parser = argparse.ArgumentParser(description='Process URDF and create sphere controller data.')
    parser.add_argument('--robot_path', type=str, required=True, help='Path to URDF file')
    parser.add_argument('--base_link', type=str, default='base_link', help='Base link name')
    parser.add_argument('--verbose', action='store_true', help='Enable verbose output and visualizations')
    parser.add_argument('--left', action='store_true', help='Process as left hand (invert anchors)')
    parser.add_argument('--correct_axes', action='store_true', help='Correct joint axes to link centers')
    return parser


if __name__ == "__main__":
    args = make_parser().parse_args()

    robot_dir = args.robot_path
    verbosity = args.verbose
    left = args.left
    base_link = args.base_link
    correct_joint_axis = args.correct_axes

    # === Configuration ===
    anchor_values = np.array([-0.7, -0.3, 0.0, 0.3, 1.571])  # thumb special
    merge_fingers = [0, 1]
    control_sphere_samples = 20000
    num_skeleton_points = 200

    if left:
        anchor_values = -np.array(list(reversed(anchor_values)))
        merge_fingers = [4 - x for x in merge_fingers]

    robot_folder = os.path.dirname(robot_dir)

    # Load config.json
    config_path = os.path.join(robot_folder, "config.json")
    try:
        with open(config_path, 'r') as f:
            config = json.load(f)
        q_opened = config.get("opened_dofs", None)
    except FileNotFoundError:
        print(f"Config file not found at {config_path}")
        q_opened = None

    print(f"Config file found at {config_path}")
    print()

    # === Load URDF and meshes ===
    robot = load_urdf(robot_dir)
    link_meshes = load_link_meshes(robot_dir, robot)

    if verbosity:
        print("Loaded URDF and meshes successfully.")

    # === Analyze kinematic structure ===
    joint_info, kin_chains, vertices_dict, meshes_dict = get_unfixed_kin_chains(
        robot_dir, link_meshes, base_link, correct_joint_axis
    )

    if verbosity:
        print(f"Found {len(kin_chains)} kinematic chains")
        for joint in list(joint_info.keys())[:5]:
            print(joint, ":", joint_info[joint])

    # Add fingertip sampling points
    for finger in [c for c in kin_chains if "palm_normal" not in c and "sphere_frame" not in c]:
        fingertip = finger[-1]
        z = np.linspace(0, 5, 20) * 0.01
        ft_points = np.stack((np.zeros_like(z), np.zeros_like(z), z), axis=1)
        vertices_dict[fingertip] = np.vstack((vertices_dict.get(fingertip, np.empty((0,3))), ft_points))

    # === Core processing pipeline ===
    construct_plane(vertices_dict, joint_info, kin_chains, base_link)

    # Process Type A/B/C joints
    joint_type_info = process_type_ABC_joints(
        [c for c in kin_chains if "palm_normal" not in c and "sphere_frame" not in c],
        joint_info, kin_chains, vertices_dict, meshes_dict, q_opened
    )

    # Solve IK for max reach
    q_max_dict = solve_inverse_kinematics(
        [c for c in kin_chains if "palm_normal" not in c and "sphere_frame" not in c],
        joint_info, kin_chains, vertices_dict, joint_type_info, q_opened
    )

    # Build sphere
    n_axis_points = construct_sphere_with_palm(
        vertices_dict, joint_info, kin_chains, q_opened, q_max_dict, base_link
    )

    # Compute main joints and finger order
    main_joints, theta_ranges = compute_main_joints(
        joint_info, kin_chains, q_opened, q_max_dict, joint_type_info, base_link
    )

    theta_alignment, finger_chain_controller_indices = compute_finger_chain_order_translation(
        joint_info, kin_chains, theta_ranges, q_opened, left, base_link
    )

    # Merge fingers logic
    chain_indices = []
    c = 0
    for chain in [ch for ch in kin_chains if "palm_normal" not in ch and "sphere_frame" not in ch]:
        if (merge_fingers[0] not in finger_chain_controller_indices) and finger_chain_controller_indices[c] == merge_fingers[1]:
            chain_indices.append(merge_fingers)
        elif (merge_fingers[1] not in finger_chain_controller_indices) and finger_chain_controller_indices[c] == merge_fingers[0]:
            chain_indices.append(merge_fingers)
        else:
            chain_indices.append([finger_chain_controller_indices[c]])
        c += 1

    print("Finger Chain order:", finger_chain_controller_indices)
    print("Finger Chain indices:", chain_indices)

    # Process bounding boxes
    process_bounding_boxes(
        [c for c in kin_chains if "palm_normal" not in c and "sphere_frame" not in c],
        vertices_dict, joint_info, joint_type_info, q_opened, main_joints
    )

    # Generate fingerprints
    finger_print_dict = spherical_raycasts(
        meshes_dict, joint_info, kin_chains, q_max_dict, 1000, base_link
    )

    fp_7_point_sample = compute_7_point_sample_skeleton(
        finger_print_dict, joint_info, kin_chains, q_opened
    )

    # Build lookup tables for sphere control
    sphere_control_dict = {}
    projected_fp_dict = {}
    q_sphere = q_opened.copy() if q_opened else {}
    q_max_phi = q_opened.copy() if q_opened else {}

    c = 0
    theta_min = np.zeros_like(anchor_values)
    theta_max = np.zeros_like(anchor_values)

    finger_chains = [c for c in kin_chains if "palm_normal" not in c and "sphere_frame" not in c]

    for joint in main_joints:
        chain_idx = chain_indices[c]
        chain_anchor_values = anchor_values[chain_idx]
        merge_anchor = None

        if len(chain_idx) == 1 and chain_idx[0] in merge_fingers:
            m_list = merge_fingers.copy()
            m_list.remove(chain_idx[0])
            merge_anchor = anchor_values[m_list]

        projected_fp_sample, sphere_control, q_finger, q_phi, min_offset, max_offset = build_lookup_dict_with_fp_and_phi(
            fp_7_point_sample,
            joint,
            q_opened,
            joint_info,
            kin_chains,
            joint_type_info,
            main_joints,
            chain_idx,
            merge_anchor,
            anchor_override=chain_anchor_values,
            realign_sphere=False,
            resolution=0.005,
            N=200,
            base_link=base_link,
        )

        sphere_control_dict.update(sphere_control)
        projected_fp_dict.update(projected_fp_sample)

        chain = next((ch for ch in finger_chains if joint in ch), None)
        if chain:
            theta_min[chain_idx] = min_offset
            theta_max[chain_idx] = max_offset
            for j in chain[1:-1]:
                q_sphere[j] = q_finger.get(j, q_sphere.get(j, 0))
                q_max_phi[j] = q_phi.get(j, q_max_phi.get(j, 0))
        c += 1

    # Process Type D joints
    type_D_info = process_type_D_joints(finger_chains, main_joints, joint_type_info)
    for joint in type_D_info:
        joint_type_info[joint].update(type_D_info[joint])

    # Final fingerprints at q_sphere
    finger_print_dict_final = spherical_raycasts(
        meshes_dict, joint_info, kin_chains, q_sphere, control_sphere_samples, base_link
    )

    # === Save final structured data ===
    cik_json_path = os.path.join(robot_folder, "sphere_cik.json")

    cik_structured = {
        "joint_info": joint_info,
        "kin_chains": kin_chains,
        "chain_order": finger_chain_controller_indices,
        "chain_indices": chain_indices,
        "joint_type_info": joint_type_info,
        "q_open_palm": q_opened,
        "q_sphere": q_sphere,
        "min_offsets": theta_min.tolist(),
        "max_offsets": theta_max.tolist(),
        "finger_print_dict": {
            k: {
                "points": v["points"].tolist() if hasattr(v["points"], "tolist") else v["points"],
                "theta": v["theta"].tolist() if hasattr(v["theta"], "tolist") else v["theta"],
                "phi": v["phi"].tolist() if hasattr(v["phi"], "tolist") else v["phi"],
            } for k, v in finger_print_dict_final.items() if v
        },
        "fp_sample_7": {k: v.tolist() if hasattr(v, "tolist") else v for k, v in fp_7_point_sample.items() if len(v) > 0},
        "sphere_control": sphere_control_dict
    }

    save_to_json(cik_json_path, cik_structured)
    print(f"\nSaved cleaned sphere_cik.json to {cik_json_path}")

    if verbosity:
        print("Processing complete. Verbose visualizations were available but disabled in cleaned version.")