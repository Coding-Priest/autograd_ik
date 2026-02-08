#! /usr/bin/env python3

import pybullet as p
import pybullet_data
import torch
import time

# --- 3D Forward Kinematics in PyTorch ---
def pytorch_fk(thetas, link_lengths):
    """Calculates 3D end-effector position using PyTorch."""
    current_rot_matrix = torch.eye(3)
    last_pos = torch.tensor([0.0, 0.0, 0.0])
    
    # Pre-define some constants as tensors to keep torch.stack happy
    zero = torch.tensor(0.0)
    one = torch.tensor(1.0)
    
    for i in range(len(thetas)):
        angle = thetas[i]
        c, s = torch.cos(angle), torch.sin(angle)
        
        if i % 2 == 0: # Z-axis rotation
            # Each element in the stack must be a tensor
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
        # Extend along the local Z axis of the current frame
        z_axis = torch.tensor([0.0, 0.0, link_lengths[i]])
        offset = torch.matmul(current_rot_matrix, z_axis)
        last_pos = last_pos + offset
        
    return last_pos

# --- PyBullet Setup ---
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, 0)
planeId = p.loadURDF("plane.urdf")

# Target Marker
target_visual = p.createVisualShape(p.GEOM_SPHERE, radius=0.05, rgbaColor=[1, 0, 0, 1])
target_id = p.createMultiBody(0, -1, target_visual, basePosition=[0.5, 0.5, 0.5])

# Debug Sliders
target_x = p.addUserDebugParameter("Target X", -1, 1, 0.5)
target_y = p.addUserDebugParameter("Target Y", -1, 1, 0.2)
target_z = p.addUserDebugParameter("Target Z", 0, 1, 0.5)

# --- Robot Parameters ---
link_lengths = torch.tensor([0.2, 0.2, 0.2, 0.15, 0.1, 0.05])
thetas = torch.zeros(6, requires_grad=True)
optimizer = torch.optim.Adam([thetas], lr=0.05)

# --- Main Loop ---
try:
    while p.isConnected():
        # 1. Update Target
        tx = p.readUserDebugParameter(target_x)
        ty = p.readUserDebugParameter(target_y)
        tz = p.readUserDebugParameter(target_z)
        target_vec = torch.tensor([tx, ty, tz])
        p.resetBasePositionAndOrientation(target_id, [tx, ty, tz], [0, 0, 0, 1])

        # 2. Optimization Step
        optimizer.zero_grad()
        eef_pos = pytorch_fk(thetas, link_lengths)
        
        # Loss: MSE + small regularization to keep joints from spinning wildly
        dist_loss = torch.nn.functional.mse_loss(eef_pos, target_vec)
        reg_loss = 0.001 * torch.sum(thetas**2)
        total_loss = dist_loss + reg_loss
        
        total_loss.backward()
        optimizer.step()

        # 3. Visualization
        p.removeAllUserDebugItems()
        # Draw the target line for clarity
        p.addUserDebugLine([0,0,0], target_vec.tolist(), [1, 0, 0], 1)
        
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
                next_pos = current_pos + torch.matmul(current_rot, torch.tensor([0.0, 0.0, link_lengths[i]]))
                
                # Draw Cyan Arm
                p.addUserDebugLine(current_pos.tolist(), next_pos.tolist(), [0, 1, 1], 8)
                current_pos = next_pos

        p.stepSimulation()
        time.sleep(1./60.)
except p.error:
    pass
finally:
    p.disconnect()
