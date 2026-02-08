#! /usr/bin/env python3

import pybullet as p
import pybullet_data
import torch
import time
import math
import numpy as np

# --- 1. Math Helpers ---

def rot_matrix_to_quat(rot_matrix):
    """Converts 3x3 rot matrix to PyBullet quaternion [x,y,z,w]"""
    m = rot_matrix.detach().cpu().numpy()
    tr = m[0][0] + m[1][1] + m[2][2]
    if tr > 0:
        S = math.sqrt(tr + 1.0) * 2
        w = 0.25 * S
        x = (m[2][1] - m[1][2]) / S
        y = (m[0][2] - m[2][0]) / S
        z = (m[1][0] - m[0][1]) / S
    elif (m[0][0] > m[1][1]) and (m[0][0] > m[2][2]):
        S = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2
        w = (m[2][1] - m[1][2]) / S
        x = 0.25 * S
        y = (m[0][1] + m[1][0]) / S
        z = (m[0][2] + m[2][0]) / S
    elif m[1][1] > m[2][2]:
        S = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2
        w = (m[0][2] - m[2][0]) / S
        x = (m[0][1] + m[1][0]) / S
        y = 0.25 * S
        z = (m[1][2] + m[2][1]) / S
    else:
        S = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2
        w = (m[1][0] - m[0][1]) / S
        x = (m[0][2] + m[2][0]) / S
        y = (m[1][2] + m[2][1]) / S
        z = 0.25 * S
    return [x, y, z, w]

def rpy_to_matrix(roll, pitch, yaw):
    """Creates a 3x3 rotation matrix from RPY."""
    rx = torch.eye(3)
    c, s = torch.cos(roll), torch.sin(roll)
    rx[1,1], rx[1,2] = c, -s
    rx[2,1], rx[2,2] = s, c
    
    ry = torch.eye(3)
    c, s = torch.cos(pitch), torch.sin(pitch)
    ry[0,0], ry[0,2] = c, s
    ry[2,0], ry[2,2] = -s, c
    
    rz = torch.eye(3)
    c, s = torch.cos(yaw), torch.sin(yaw)
    rz[0,0], rz[0,1] = c, -s
    rz[1,0], rz[1,1] = s, c
    return torch.matmul(rz, torch.matmul(ry, rx))

# --- 2. PyTorch Forward Kinematics (Updated for Collision Info) ---

def pytorch_fk(thetas, link_lengths):
    """
    Returns:
    - eef_pos: End effector position
    - eef_rot: End effector rotation matrix
    - segment_points: List of [start_pos, mid_pos, end_pos] for every link
    """
    current_rot_matrix = torch.eye(3)
    current_pos = torch.tensor([0.0, 0.0, 0.0])
    
    zero = torch.tensor(0.0)
    one = torch.tensor(1.0)
    
    # Store points for collision detection
    # Dimensions: (Num_Links, 3_points_per_link, 3_xyz)
    segment_points = [] 

    for i in range(len(thetas)):
        angle = thetas[i]
        c, s = torch.cos(angle), torch.sin(angle)
        
        # Rotation Logic
        if i % 2 == 0:
            rot = torch.stack([torch.stack([c, -s, zero]), torch.stack([s, c, zero]), torch.stack([zero, zero, one])])
        else:
            rot = torch.stack([torch.stack([c, zero, s]), torch.stack([zero, one, zero]), torch.stack([-s, zero, c])])
            
        current_rot_matrix = torch.matmul(current_rot_matrix, rot)
        
        # Calculate Link Vector
        z_axis = torch.tensor([0.0, 0.0, link_lengths[i]])
        offset = torch.matmul(current_rot_matrix, z_axis)
        next_pos = current_pos + offset
        
        # Generate 3 Collision Test Points (Start, Middle, End)
        p_start = current_pos
        p_end = next_pos
        p_mid = (p_start + p_end) / 2.0
        
        segment_points.append(torch.stack([p_start, p_mid, p_end]))
        
        current_pos = next_pos
        
    return current_pos, current_rot_matrix, torch.stack(segment_points)

def compute_collision_loss(segment_points):
    """
    Calculates penalties for:
    1. Self-collision (non-adjacent links touching)
    2. Ground collision (going below z=0)
    """
    loss = torch.tensor(0.0)
    num_links = segment_points.shape[0]
    
    # 1. Ground Collision
    # Check all Z coordinates of all points
    all_points_flat = segment_points.view(-1, 3) # Flatten to [N_points, 3]
    z_coords = all_points_flat[:, 2]
    
    # If Z < 0.05 (floor buffer), penalize
    floor_violation = torch.relu(0.05 - z_coords) 
    loss = loss + torch.sum(floor_violation ** 2) * 50.0

    # 2. Self Collision
    # We check pairs of links (i, j) where |i - j| > 1
    # Adjacent links (like 0 and 1) naturally touch at the joint, so we skip them.
    
    safe_dist = 0.08 # Combined radius of two capsules (0.04 + 0.04)
    
    for i in range(num_links):
        for j in range(i + 2, num_links): # Skip i+1 (adjacent)
            # Get points for link i and link j
            # Shape: [3, 3] (3 points, xyz)
            points_i = segment_points[i]
            points_j = segment_points[j]
            
            # Calculate pairwise distance between all points in Link i and Link j
            # Simple broadcasting: expand dims to form matrix
            # p_i: [3, 1, 3], p_j: [1, 3, 3]
            dist = torch.norm(points_i.unsqueeze(1) - points_j.unsqueeze(0), dim=2)
            
            # If distance < safe_dist, apply penalty
            # ReLU(safe - actual) gives 0 if safe < actual (no collision)
            # gives (safe - actual) if collision
            violation = torch.relu(safe_dist - dist)
            
            loss = loss + torch.sum(violation ** 2) * 100.0 # High weight
            
    return loss

# --- 3. Setup ---
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, 0)
planeId = p.loadURDF("plane.urdf")

# Sliders
target_x = p.addUserDebugParameter("Target X", -1, 1, 0.4)
target_y = p.addUserDebugParameter("Target Y", -1, 1, 0.2)
target_z = p.addUserDebugParameter("Target Z", 0.1, 1, 0.5)
target_roll = p.addUserDebugParameter("Target Roll", -3.14, 3.14, 0)
target_pitch = p.addUserDebugParameter("Target Pitch", -3.14, 3.14, 0)
target_yaw = p.addUserDebugParameter("Target Yaw", -3.14, 3.14, 0)

# Robot Config
link_lengths_vals = [0.2, 0.2, 0.2, 0.15, 0.1, 0.05]
link_lengths = torch.tensor(link_lengths_vals)
thetas = torch.zeros(6, requires_grad=True)
# Add small noise to initial thetas to avoid "perfectly straight" singularity 
# where gradients might be zero for self-collision
with torch.no_grad():
    thetas += 0.01 * torch.randn(6)

optimizer = torch.optim.Adam([thetas], lr=0.05) 

# Visuals
robot_visual_ids = []
colors = [[0, 0.8, 1, 1], [0, 0.5, 1, 1]] 
for i, length in enumerate(link_lengths_vals):
    radius = 0.035 # Slightly fatter visuals
    visual_shape_id = p.createVisualShape(p.GEOM_CAPSULE, radius=radius, length=length, rgbaColor=colors[i % 2], visualFramePosition=[0,0,0])
    body_id = p.createMultiBody(baseMass=0, baseVisualShapeIndex=visual_shape_id, basePosition=[0,0,0], baseOrientation=[0,0,0,1])
    robot_visual_ids.append(body_id)

def draw_coordinate_frame(pos, rot_mat, length=0.15, duration=0.1):
    origin = np.array(pos)
    rot = np.array(rot_mat)
    p.addUserDebugLine(origin, origin + rot @ [length, 0, 0], [1, 0, 0], lineWidth=3, lifeTime=duration)
    p.addUserDebugLine(origin, origin + rot @ [0, length, 0], [0, 1, 0], lineWidth=3, lifeTime=duration)
    p.addUserDebugLine(origin, origin + rot @ [0, 0, length], [0, 0, 1], lineWidth=3, lifeTime=duration)

# --- 4. Main Loop ---
print("Simulating with Self-Collision Avoidance...")
try:
    while p.isConnected():
        # Read Inputs
        tx, ty, tz = [p.readUserDebugParameter(i) for i in [target_x, target_y, target_z]]
        tr, tp, tyaw = [p.readUserDebugParameter(i) for i in [target_roll, target_pitch, target_yaw]]
        
        target_pos = torch.tensor([tx, ty, tz])
        target_rot_matrix = rpy_to_matrix(torch.tensor(tr), torch.tensor(tp), torch.tensor(tyaw))

        # Optimization
        optimizer.zero_grad()
        eef_pos, eef_rot, segment_points = pytorch_fk(thetas, link_lengths)
        
        # Losses
        loss_pos = torch.nn.functional.mse_loss(eef_pos, target_pos)
        loss_rot = torch.nn.functional.mse_loss(eef_rot, target_rot_matrix)
        loss_reg = 0.0001 * torch.sum(thetas**2) # Keep joints efficient
        loss_col = compute_collision_loss(segment_points) # <-- NEW: Collision Loss
        
        # Combined Loss
        # We weigh collision very high so it overrides reaching the target if unsafe
        total_loss = loss_pos + (0.5 * loss_rot) + loss_reg + loss_col
        
        total_loss.backward()
        optimizer.step()

        # Visualization
        draw_coordinate_frame(target_pos.tolist(), target_rot_matrix.detach().numpy(), duration=1./15.)
        
        with torch.no_grad():
            current_rot = torch.eye(3)
            current_pos = torch.tensor([0.0, 0.0, 0.0])
            for i in range(len(thetas)):
                # FK Re-calculation for Viz (Standard)
                angle = thetas[i]
                c, s = torch.cos(angle), torch.sin(angle)
                if i % 2 == 0:
                    rot = torch.tensor([[c, -s, 0], [s, c, 0], [0, 0, 1]])
                else:
                    rot = torch.tensor([[c, 0, s], [0, 1, 0], [-s, 0, c]])
                current_rot = torch.matmul(current_rot, rot)
                link_vec = torch.matmul(current_rot, torch.tensor([0.0, 0.0, link_lengths[i]]))
                next_pos = current_pos + link_vec
                
                midpoint = (current_pos + next_pos) / 2.0
                quat = rot_matrix_to_quat(current_rot)
                p.resetBasePositionAndOrientation(robot_visual_ids[i], midpoint.tolist(), quat)
                current_pos = next_pos

        p.stepSimulation()
        time.sleep(1./60.)
        
except p.error:
    pass
finally:
    p.disconnect()
