import pygame
import torch

# --- Configuration ---
WIDTH, HEIGHT = 800, 600
ORIGIN = torch.tensor([WIDTH // 2, HEIGHT // 2], dtype=torch.float32)
# Lengths of the three links
L1, L2, L3 = 120.0, 100.0, 80.0
LEARNING_RATE = 0.05

# --- Initialize PyTorch Parameters ---
# Now using 3 angles (thetas)
thetas = torch.tensor([0.1, 0.1, 0.1], requires_grad=True)
optimizer = torch.optim.RMSprop([thetas], lr=LEARNING_RATE)

def forward_kinematics(t):
    # Joint 1 position (Base to Joint 1)
    x1 = ORIGIN[0] + L1 * torch.cos(t[0])
    y1 = ORIGIN[1] + L1 * torch.sin(t[0])
    j1 = torch.stack([x1, y1])
    
    # Joint 2 position (Joint 1 to Joint 2)
    x2 = x1 + L2 * torch.cos(t[0] + t[1])
    y2 = y1 + L2 * torch.sin(t[0] + t[1])
    j2 = torch.stack([x2, y2])
    
    # End Effector position (Joint 2 to EEF)
    x3 = x2 + L3 * torch.cos(t[0] + t[1] + t[2])
    y3 = y2 + L3 * torch.sin(t[0] + t[1] + t[2])
    eef = torch.stack([x3, y3])
    
    return j1, j2, eef

# --- Pygame Setup ---
pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("3-DOF Autograd Inverse Kinematics")
clock = pygame.time.Clock()
target_pos = torch.tensor([WIDTH // 2 + 150, HEIGHT // 2 - 150], dtype=torch.float32)

running = True
while running:
    screen.fill((25, 25, 35)) # Darker slate background
    
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.MOUSEBUTTONDOWN or pygame.mouse.get_pressed()[0]:
            mouse_x, mouse_y = pygame.mouse.get_pos()
            target_pos = torch.tensor([mouse_x, mouse_y], dtype=torch.float32)

    # --- IK Optimization Step ---
    optimizer.zero_grad()
    j1_pos, j2_pos, eef_pos = forward_kinematics(thetas)
    
    # Loss: Simple MSE between EEF and Target
    loss = torch.nn.functional.mse_loss(eef_pos, target_pos)
    
    if loss > 0.01:
        loss.backward()
        optimizer.step()

    # --- Drawing ---
    # Draw Target (Red)
    pygame.draw.circle(screen, (255, 80, 80), target_pos.detach().numpy().astype(int), 10, 2)
    
    # Convert to numpy for drawing
    pts = [
        ORIGIN.numpy().astype(int),
        j1_pos.detach().numpy().astype(int),
        j2_pos.detach().numpy().astype(int),
        eef_pos.detach().numpy().astype(int)
    ]
    
    # Draw Links (Gray lines)
    pygame.draw.lines(screen, (180, 180, 180), False, pts, 6)
    
    # Draw Joint Hinges
    colors = [(50, 255, 255), (50, 255, 255), (50, 255, 255), (255, 255, 50)]
    for i, p in enumerate(pts):
        pygame.draw.circle(screen, colors[i], p, 7)

    pygame.display.flip()
    clock.tick(60)

pygame.quit()