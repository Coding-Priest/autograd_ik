import pybullet as p
import pybullet_data
import torch
import time
import math
import numpy as np

# --- Helper: Rotation Matrix to Quaternion ---
def rot_matrix_to_quat(rot_matrix):
    """
    Converts a 3x3 PyTorch rotation matrix to a list [x, y, z, w] for PyBullet.
    """
    # Ensure it's on CPU and numpy
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

# --- 3D Forward Kinematics in PyTorch ---
def pytorch_fk(thetas, link_lengths):
    """Calculates 3D end-effector position using PyTorch."""
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
        
    return last_pos

# --- PyBullet Setup ---
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, 0)
planeId = p.loadURDF("plane.urdf")

# Target Marker
target_visual = p.createVisualShape(p.GEOM_SPHERE, radius=0.05, rgbaColor=[1, 0, 0, 0.6])
target_id = p.createMultiBody(0, -1, target_visual, basePosition=[0.5, 0.5, 0.5])

# Debug Sliders
target_x = p.addUserDebugParameter("Target X", -1, 1, 0.5)
target_y = p.addUserDebugParameter("Target Y", -1, 1, 0.2)
target_z = p.addUserDebugParameter("Target Z", 0, 1, 0.5)

# --- Robot Parameters ---
link_lengths_vals = [0.2, 0.2, 0.2, 0.15, 0.1, 0.05]
link_lengths = torch.tensor(link_lengths_vals)
thetas = torch.zeros(6, requires_grad=True)
optimizer = torch.optim.Adam([thetas], lr=0.05)

# --- VISUALIZATION SETUP: Create Solid Bodies ---
robot_visual_ids = []
colors = [[0, 0.8, 1, 1], [0, 0.5, 1, 1]] # Alternating Blues

for i, length in enumerate(link_lengths_vals):
    # Create a capsule
    # Note: p.GEOM_CAPSULE creates a shape centered at 0,0,0
    radius = 0.03
    visual_shape_id = p.createVisualShape(
        shapeType=p.GEOM_CAPSULE,
        radius=radius,
        length=length,
        rgbaColor=colors[i % 2],
        visualFramePosition=[0, 0, 0] # Local visual offset
    )
    # Create the body (no mass, static)
    body_id = p.createMultiBody(
        baseMass=0, 
        baseVisualShapeIndex=visual_shape_id, 
        basePosition=[0, 0, 0],
        baseOrientation=[0, 0, 0, 1]
    )
    robot_visual_ids.append(body_id)

# --- Main Loop ---
print("Simulating...")
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
        dist_loss = torch.nn.functional.mse_loss(eef_pos, target_vec)
        reg_loss = 0.001 * torch.sum(thetas**2)
        total_loss = dist_loss + reg_loss
        total_loss.backward()
        optimizer.step()

        # 3. Visualization (Update Bodies)
        # We assume the debug lines were removed to stop flickering.
        # Now we calculate the exact position for each capsule.
        
        with torch.no_grad():
            current_rot = torch.eye(3)
            current_pos = torch.tensor([0.0, 0.0, 0.0])
            
            for i in range(len(thetas)):
                angle = thetas[i]
                c, s = torch.cos(angle), torch.sin(angle)
                
                # Apply Rotation
                if i % 2 == 0:
                    rot = torch.tensor([[c, -s, 0], [s, c, 0], [0, 0, 1]])
                else:
                    rot = torch.tensor([[c, 0, s], [0, 1, 0], [-s, 0, c]])
                
                current_rot = torch.matmul(current_rot, rot)
                
                # Calculate vector for this link
                link_vec = torch.matmul(current_rot, torch.tensor([0.0, 0.0, link_lengths[i]]))
                next_pos = current_pos + link_vec
                
                # --- UPDATE PYBULLET BODY ---
                # 1. Position: PyBullet bodies are centered. 
                # The link goes from current_pos to next_pos.
                # So the body center is the midpoint.
                midpoint = (current_pos + next_pos) / 2.0
                
                # 2. Orientation: Convert Rotation Matrix to Quaternion
                quat = rot_matrix_to_quat(current_rot)
                
                # Move the capsule to the new calculation
                p.resetBasePositionAndOrientation(
                    robot_visual_ids[i], 
                    midpoint.tolist(), 
                    quat
                )
                
                current_pos = next_pos

        p.stepSimulation()
        # Slightly faster sleep for smoother animation
        time.sleep(1./120.) 
        
except p.error:
    pass
finally:
    p.disconnect()