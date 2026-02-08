import pybullet as p
import pybullet_data
import torch
import time
import numpy as np

# --- 3D Forward Kinematics in PyTorch ---
def pytorch_fk(thetas, link_lengths):
    """Calculates 3D end-effector position using PyTorch."""
    # Start at base
    curr_pos = torch.tensor([0.0, 0.0, 0.0])
    
    # We'll simplify this 6-DOF arm: 
    # Joints alternate between rotating around Z and Y axes
    # Joint 0: Z, Joint 1: Y, Joint 2: Y, Joint 3: Z, Joint 4: Y, Joint 5: X
    
    # This is a simplified transformation chain
    # In a real robot, we'd use full 4x4 Homogeneous Matrices
    x, y, z = 0.0, 0.0, 0.0
    accum_angle_y = 0.0
    accum_angle_z = 0.0
    
    # Let's build a vertical chain for visualization
    # Joint 0 (Yaw)
    z_rot = thetas[0]
    # Joint 1 (Pitch)
    y_rot = thetas[1]
    
    # Simplified 3D position logic for a 6-link chain
    # For brevity, we'll simulate a 3D 'snake' reaching out
    pos = torch.zeros(3)
    current_rot_matrix = torch.eye(3)
    
    last_pos = torch.tensor([0.0, 0.0, 0.0])
    
    for i in range(len(thetas)):
        # Create rotation matrix for this joint (alternating Y and Z)
        angle = thetas[i]
        c, s = torch.cos(angle), torch.sin(angle)
        
        if i % 2 == 0: # Z-axis rotation
            rot = torch.stack([
                torch.stack([c, -s, 0.0]),
                torch.stack([s, c, 0.0]),
                torch.stack([0.0, 0.0, 1.0])
            ])
        else: # Y-axis rotation
            rot = torch.stack([
                torch.stack([c, 0.0, s]),
                torch.stack([0.0, 1.0, 0.0]),
                torch.stack([-s, 0.0, c])
            ])
            
        current_rot_matrix = torch.matmul(current_rot_matrix, rot)
        # Extend along the local Z axis of the current frame
        offset = torch.matmul(current_rot_matrix, torch.tensor([0.0, 0.0, link_lengths[i]]))
        last_pos = last_pos + offset
        
    return last_pos

# --- PyBullet Setup ---
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, 0) # No gravity for pure kinematics
planeId = p.loadURDF("plane.urdf")

# Create a simple visual marker for the target
target_visual = p.createVisualShape(p.GEOM_SPHERE, radius=0.05, rgbaColor=[1, 0, 0, 1])
target_id = p.createMultiBody(0, -1, target_visual, basePosition=[0.5, 0.5, 0.5])

# Add Sliders to move the target in 3D
target_x = p.addUserDebugParameter("Target X", -1, 1, 0.5)
target_y = p.addUserDebugParameter("Target Y", -1, 1, 0.2)
target_z = p.addUserDebugParameter("Target Z", 0, 1, 0.5)

# --- Robot Setup (Visual only) ---
# We'll use debug lines to draw our 6-DOF arm in PyBullet
link_lengths = torch.tensor([0.2, 0.2, 0.2, 0.15, 0.1, 0.05])
thetas = torch.zeros(6, requires_grad=True)
optimizer = torch.optim.Adam([thetas], lr=0.05)

line_ids = []

while True:
    # 1. Update Target Position from Sliders
    tx = p.readUserDebugParameter(target_x)
    ty = p.readUserDebugParameter(target_y)
    tz = p.readUserDebugParameter(target_z)
    target_vec = torch.tensor([tx, ty, tz])
    p.resetBasePositionAndOrientation(target_id, [tx, ty, tz], [0, 0, 0, 1])

    # 2. Optimization Step
    optimizer.zero_grad()
    
    # We need the full chain for drawing, but only the tip for loss
    eef_pos = pytorch_fk(thetas, link_lengths)
    
    loss = torch.nn.functional.mse_loss(eef_pos, target_vec)
    
    # Regularization to keep angles within reasonable bounds
    reg = 0.001 * torch.sum(thetas**2)
    (loss + reg).backward()
    optimizer.step()

    # 3. Visualization (Draw the arm using lines)
    p.removeAllUserDebugItems()
    
    # Re-calculate FK chain for drawing
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
            
            # Draw link
            p.addUserDebugLine(current_pos.tolist(), next_pos.tolist(), [0, 1, 1], 5)
            current_pos = next_pos

    p.stepSimulation()
    time.sleep(1./240.)