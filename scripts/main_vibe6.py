#! /usr/bin/env python3

import pybullet as p
import pybullet_data
import torch
import time
import math
import numpy as np

# --- Helper: Rotation Matrix to Quaternion ---
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

# --- Helper: Euler (RPY) to Rotation Matrix ---
def rpy_to_matrix(roll, pitch, yaw):
    """Creates a 3x3 rotation matrix from RPY (XYZ convention) using PyTorch."""
    # Rotation Matrices for X, Y, Z
    
    # Roll (X-axis)
    rx = torch.eye(3)
    c, s = torch.cos(roll), torch.sin(roll)
    rx[1,1], rx[1,2] = c, -s
    rx[2,1], rx[2,2] = s, c
    
    # Pitch (Y-axis)
    ry = torch.eye(3)
    c, s = torch.cos(pitch), torch.sin(pitch)
    ry[0,0], ry[0,2] = c, s
    ry[2,0], ry[2,2] = -s, c
    
    # Yaw (Z-axis)
    rz = torch.eye(3)
    c, s = torch.cos(yaw), torch.sin(yaw)
    rz[0,0], rz[0,1] = c, -s
    rz[1,0], rz[1,1] = s, c
    
    # Combined Rotation R = Rz * Ry * Rx (Standard intrinsic/extrinsic order)
    # Note: Order depends on convention, Rz @ Ry @ Rx is common for global frame
    return torch.matmul(rz, torch.matmul(ry, rx))

# --- 3D Forward Kinematics in PyTorch ---
def pytorch_fk(thetas, link_lengths):
    """Returns both position and rotation matrix of end effector."""
    current_rot_matrix = torch.eye(3)
    last_pos = torch.tensor([0.0, 0.0, 0.0])
    
    zero = torch.tensor(0.0)
    one = torch.tensor(1.0)
    
    for i in range(len(thetas)):
        angle = thetas[i]
        c, s = torch.cos(angle), torch.sin(angle)
        
        if i % 2 == 0: # Z-axis rotation
            rot = torch.stack([
                torch.stack([c, -s, zero]),
                torch.stack([s, c, zero]),
                torch.stack([zero, zero, one])
            ])
        else: # Y-axis rotation
            rot = torch.stack([
                torch.stack([c, zero, s]),
                torch.stack([zero, one, zero]),
                torch.stack([-s, zero, c])
            ])
            
        current_rot_matrix = torch.matmul(current_rot_matrix, rot)
        z_axis = torch.tensor([0.0, 0.0, link_lengths[i]])
        offset = torch.matmul(current_rot_matrix, z_axis)
        last_pos = last_pos + offset
        
    return last_pos, current_rot_matrix

# --- PyBullet Setup ---
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, 0)
planeId = p.loadURDF("plane.urdf")

# --- Sliders ---
# Position
target_x = p.addUserDebugParameter("Target X", -1, 1, 0.5)
target_y = p.addUserDebugParameter("Target Y", -1, 1, 0.2)
target_z = p.addUserDebugParameter("Target Z", 0, 1, 0.5)
# Orientation (RPY)
target_roll = p.addUserDebugParameter("Target Roll", -3.14, 3.14, 0)
target_pitch = p.addUserDebugParameter("Target Pitch", -3.14, 3.14, 0)
target_yaw = p.addUserDebugParameter("Target Yaw", -3.14, 3.14, 0)

# --- Robot Parameters ---
link_lengths_vals = [0.2, 0.2, 0.2, 0.15, 0.1, 0.05]
link_lengths = torch.tensor(link_lengths_vals)
# 6 Joints = 6 Degrees of Freedom (enough for Position + Orientation)
thetas = torch.zeros(6, requires_grad=True)
# slightly higher learning rate for faster convergence
optimizer = torch.optim.Adam([thetas], lr=0.08) 

# --- VISUALIZATION SETUP ---
robot_visual_ids = []
colors = [[0, 0.8, 1, 1], [0, 0.5, 1, 1]] 

for i, length in enumerate(link_lengths_vals):
    radius = 0.03
    visual_shape_id = p.createVisualShape(p.GEOM_CAPSULE, radius=radius, length=length, rgbaColor=colors[i % 2], visualFramePosition=[0,0,0])
    body_id = p.createMultiBody(baseMass=0, baseVisualShapeIndex=visual_shape_id, basePosition=[0,0,0], baseOrientation=[0,0,0,1])
    robot_visual_ids.append(body_id)

def draw_coordinate_frame(pos, rot_mat, length=0.15, duration=0.1):
    """Draws RGB axes at a given position and rotation."""
    origin = np.array(pos)
    rot = np.array(rot_mat) # 3x3
    
    # X axis (Red)
    x_vec = rot @ np.array([length, 0, 0])
    p.addUserDebugLine(origin, origin + x_vec, [1, 0, 0], lineWidth=3, lifeTime=duration)
    
    # Y axis (Green)
    y_vec = rot @ np.array([0, length, 0])
    p.addUserDebugLine(origin, origin + y_vec, [0, 1, 0], lineWidth=3, lifeTime=duration)
    
    # Z axis (Blue)
    z_vec = rot @ np.array([0, 0, length])
    p.addUserDebugLine(origin, origin + z_vec, [0, 0, 1], lineWidth=3, lifeTime=duration)


# --- Main Loop ---
print("Simulating...")
try:
    while p.isConnected():
        # 1. Read Inputs
        tx = p.readUserDebugParameter(target_x)
        ty = p.readUserDebugParameter(target_y)
        tz = p.readUserDebugParameter(target_z)
        tr = p.readUserDebugParameter(target_roll)
        tp = p.readUserDebugParameter(target_pitch)
        tyaw = p.readUserDebugParameter(target_yaw)
        
        # 2. Setup Target Tensors
        target_pos = torch.tensor([tx, ty, tz])
        target_rot_matrix = rpy_to_matrix(
            torch.tensor(tr), torch.tensor(tp), torch.tensor(tyaw)
        )

        # 3. Optimization Step
        optimizer.zero_grad()
        eef_pos, eef_rot = pytorch_fk(thetas, link_lengths)
        
        # Loss 1: Position Error (MSE)
        pos_loss = torch.nn.functional.mse_loss(eef_pos, target_pos)
        
        # Loss 2: Orientation Error (MSE between rotation matrices)
        # Minimizing element-wise difference aligns the frames
        rot_loss = torch.nn.functional.mse_loss(eef_rot, target_rot_matrix)
        
        # Total Loss (Weighted)
        # We weigh rotation slightly less to ensure it reaches position first, 
        # but 0.5 is usually a good balance.
        total_loss = pos_loss + (0.5 * rot_loss) + (0.0001 * torch.sum(thetas**2))
        
        total_loss.backward()
        optimizer.step()

        # 4. Visualization Update
        # Draw Target Coordinate Frame
        # We need to detach tensors to numpy for PyBullet visualization
        draw_coordinate_frame(
            target_pos.tolist(), 
            target_rot_matrix.detach().numpy(),
            duration=1./15. # Persist just long enough for next frame
        )
        
        with torch.no_grad():
            current_rot = torch.eye(3)
            current_pos = torch.tensor([0.0, 0.0, 0.0])
            
            for i in range(len(thetas)):
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
                
                p.resetBasePositionAndOrientation(
                    robot_visual_ids[i], 
                    midpoint.tolist(), 
                    quat
                )
                current_pos = next_pos
            
            # Optional: Draw End-Effector Frame too (to see alignment)
            # draw_coordinate_frame(current_pos.tolist(), current_rot.numpy(), length=0.1, duration=1./15.)

        p.stepSimulation()
        time.sleep(1./60.)
        
except p.error:
    pass
finally:
    p.disconnect()
