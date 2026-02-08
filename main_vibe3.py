import pygame
import torch

# --- Configuration ---
WIDTH, HEIGHT = 800, 600
ORIGIN = torch.tensor([WIDTH // 2, HEIGHT // 2], dtype=torch.float32)
# Define 6 link lengths
LINKS = torch.tensor([80.0, 70.0, 60.0, 50.0, 40.0, 30.0], dtype=torch.float32)
LEARNING_RATE = 0.04

# --- Initialize PyTorch Parameters ---
# 6 angles for 6 joints
thetas = torch.tensor([0.1] * 6, requires_grad=True)
optimizer = torch.optim.Adam([thetas], lr=LEARNING_RATE)

def forward_kinematics(t):
    positions = []
    curr_pos = ORIGIN
    curr_angle = torch.tensor(0.0)
    
    positions.append(curr_pos)
    
    for i in range(len(t)):
        curr_angle = curr_angle + t[i]
        # Calculate next joint position based on accumulated angle
        next_x = curr_pos[0] + LINKS[i] * torch.cos(curr_angle)
        next_y = curr_pos[1] + LINKS[i] * torch.sin(curr_angle)
        curr_pos = torch.stack([next_x, next_y])
        positions.append(curr_pos)
        
    return positions # Returns [Base, J1, J2, J3, J4, J5, EEF]

# --- Pygame Setup ---
pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("6-DOF Autograd Snake Arm")
clock = pygame.time.Clock()
target_pos = torch.tensor([WIDTH // 2 + 200, HEIGHT // 2], dtype=torch.float32)

running = True
while running:
    screen.fill((20, 22, 28)) # Deep space blue
    
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        # Update target on click or drag
        if pygame.mouse.get_pressed()[0]:
            mouse_x, mouse_y = pygame.mouse.get_pos()
            target_pos = torch.tensor([mouse_x, mouse_y], dtype=torch.float32)

    # --- IK Optimization Step ---
    optimizer.zero_grad()
    all_joint_positions = forward_kinematics(thetas)
    eef_pos = all_joint_positions[-1]
    
    # Distance Loss
    dist_loss = torch.nn.functional.mse_loss(eef_pos, target_pos)
    
    # Optional: Regularization (keeps the arm from over-rotating)
    # This acts like a "muscle tension" that prefers a straight-ish arm
    reg_loss = 0.01 * torch.sum(thetas**2)
    
    total_loss = dist_loss + reg_loss
    
    if dist_loss > 0.01:
        total_loss.backward()
        optimizer.step()

    # --- Drawing ---
    # Draw Target
    pygame.draw.circle(screen, (255, 100, 100), target_pos.detach().numpy().astype(int), 12, 2)
    
    # Convert all positions to numpy
    pts = [p.detach().numpy().astype(int) for p in all_joint_positions]
    
    # Draw Links
    pygame.draw.lines(screen, (150, 150, 160), False, pts, 4)
    
    # Draw Joints with a gradient feel
    for i, p in enumerate(pts):
        # Base is white, joints are cyan, end effector is gold
        color = (50, 200, 255)
        if i == 0: color = (255, 255, 255)
        if i == len(pts)-1: color = (255, 215, 0)
        
        pygame.draw.circle(screen, color, p, 5)

    pygame.display.flip()
    clock.tick(60)

pygame.quit()