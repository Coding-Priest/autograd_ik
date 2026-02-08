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

# --- 2. Kinematics & Loss ---

def pytorch_fk(thetas, link_lengths):
    """Calculates Forward Kinematics and collision points."""
    current_rot_matrix = torch.eye(3)
    current_pos = torch.tensor([0.0, 0.0, 0.0])
    
    zero = torch.tensor(0.0)
    one = torch.tensor(1.0)
    segment_points = [] 

    for i in range(len(thetas)):
        angle = thetas[i]
        c, s = torch.cos(angle), torch.sin(angle)
        
        if i % 2 == 0:
            rot = torch.stack([torch.stack([c, -s, zero]), torch.stack([s, c, zero]), torch.stack([zero, zero, one])])
        else:
            rot = torch.stack([torch.stack([c, zero, s]), torch.stack([zero, one, zero]), torch.stack([-s, zero, c])])
            
        current_rot_matrix = torch.matmul(current_rot_matrix, rot)
        z_axis = torch.tensor([0.0, 0.0, link_lengths[i]])
        offset = torch.matmul(current_rot_matrix, z_axis)
        next_pos = current_pos + offset
        
        p_start = current_pos
        p_end = next_pos
        p_mid = (p_start + p_end) / 2.0
        segment_points.append(torch.stack([p_start, p_mid, p_end]))
        
        current_pos = next_pos
        
    return current_pos, current_rot_matrix, torch.stack(segment_points)

def compute_collision_loss(segment_points):
    loss = torch.tensor(0.0)
    num_links = segment_points.shape[0]
    
    # Floor Collision
    all_points_flat = segment_points.view(-1, 3)
    z_coords = all_points_flat[:, 2]
    floor_violation = torch.relu(0.05 - z_coords) 
    loss = loss + torch.sum(floor_violation ** 2) * 50.0

    # Self Collision
    safe_dist = 0.08 
    for i in range(num_links):
        for j in range(i + 2, num_links): 
            points_i = segment_points[i]
            points_j = segment_points[j]
            dist = torch.norm(points_i.unsqueeze(1) - points_j.unsqueeze(0), dim=2)
            violation = torch.relu(safe_dist - dist)
            loss = loss + torch.sum(violation ** 2) * 100.0
            
    return loss

# --- 3. The "Brain": Solver Function ---

def solve_inverse_kinematics(target_pos, target_rot, start_thetas, link_lengths, iterations=30):
    """
    Runs an optimization loop entirely in the background (no GUI updates).
    Returns the ideal joint angles.
    """
    # Clone variables so we don't mess up the visual state during calculation
    opt_thetas = start_thetas.clone().detach().requires_grad_(True)
    optimizer = torch.optim.Adam([opt_thetas], lr=0.1) # Aggressive Learning Rate
    
    for _ in range(iterations):
        optimizer.zero_grad()
        eef_pos, eef_rot, segments = pytorch_fk(opt_thetas, link_lengths)
        
        loss_pos = torch.nn.functional.mse_loss(eef_pos, target_pos)
        loss_rot = torch.nn.functional.mse_loss(eef_rot, target_rot)
        loss_col = compute_collision_loss(segments)
        loss_reg = 0.001 * torch.sum(opt_thetas**2)
        
        total_loss = loss_pos + (0.5 * loss_rot) + loss_col + loss_reg
        total_loss.backward()
        optimizer.step()
        
    return opt_thetas.detach() # Return just the values

# --- 4. Setup ---
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

link_lengths_vals = [0.2, 0.2, 0.2, 0.15, 0.1, 0.05]
link_lengths = torch.tensor(link_lengths_vals)

# State Variables
# "current_display_thetas" is where the robot IS physically
current_display_thetas = torch.zeros(6) 
# Add some noise initially so we aren't in a singularity
current_display_thetas += 0.01 * torch.randn(6)

robot_visual_ids = []
colors = [[0, 0.8, 1, 1], [0, 0.5, 1, 1]] 
for i, length in enumerate(link_lengths_vals):
    radius = 0.035
    visual_shape_id = p.createVisualShape(p.GEOM_CAPSULE, radius=radius, length=length, rgbaColor=colors[i % 2], visualFramePosition=[0,0,0])
    body_id = p.createMultiBody(baseMass=0, baseVisualShapeIndex=visual_shape_id, basePosition=[0,0,0], baseOrientation=[0,0,0,1])
    robot_visual_ids.append(body_id)

def draw_coordinate_frame(pos, rot_mat):
    origin = np.array(pos)
    rot = np.array(rot_mat)
    p.addUserDebugLine(origin, origin + rot @ [0.15, 0, 0], [1, 0, 0], lineWidth=3, lifeTime=0.1)
    p.addUserDebugLine(origin, origin + rot @ [0, 0.15, 0], [0, 1, 0], lineWidth=3, lifeTime=0.1)
    p.addUserDebugLine(origin, origin + rot @ [0, 0, 0.15], [0, 0, 1], lineWidth=3, lifeTime=0.1)

# --- 5. Main Loop ---
print("Simulating with Interpolated Movement...")
try:
    while p.isConnected():
        # --- A. Read Inputs ---
        tx, ty, tz = [p.readUserDebugParameter(i) for i in [target_x, target_y, target_z]]
        tr, tp, tyaw = [p.readUserDebugParameter(i) for i in [target_roll, target_pitch, target_yaw]]
        
        target_pos = torch.tensor([tx, ty, tz])
        target_rot_matrix = rpy_to_matrix(torch.tensor(tr), torch.tensor(tp), torch.tensor(tyaw))
        draw_coordinate_frame([tx, ty, tz], target_rot_matrix.detach().numpy())

        # --- B. The "Brain" (Solve IK) ---
        # We start the solver from the current visual position (Warm Start)
        # This keeps the solver stable and close to the current physical state
        goal_thetas = solve_inverse_kinematics(
            target_pos, 
            target_rot_matrix, 
            current_display_thetas, 
            link_lengths, 
            iterations=5 # 20 iters per frame is fast and enough for incremental updates
        )

        # --- C. The "Body" (Interpolate) ---
        # Smoothly move current joints towards goal joints (Linear Interpolation / P-Control)
        # alpha = 0.1 means we move 10% of the way there per frame.
        alpha = 0.1
        current_display_thetas = current_display_thetas + alpha * (goal_thetas - current_display_thetas)

        # --- D. Visualization (FK Update) ---
        with torch.no_grad():
            # We re-calculate FK just for the updated visual_thetas to place the bodies
            current_rot = torch.eye(3)
            current_pos = torch.tensor([0.0, 0.0, 0.0])
            for i in range(len(current_display_thetas)):
                angle = current_display_thetas[i]
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
