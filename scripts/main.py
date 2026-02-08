#! /usr/bin/env python3

import pygame
import torch
import math

# --- Configuration ---
WIDTH, HEIGHT = 800, 600
ORIGIN = torch.tensor([WIDTH // 2, HEIGHT // 2], dtype=torch.float32)
LINK_L1 = 150.0
LINK_L2 = 100.0
LEARNING_RATE = 0.05

# --- Initialize PyTorch Parameters ---
# We start with joint angles at 0.1 radians to avoid initial zero-gradient states
thetas = torch.tensor([0.1, 0.1], requires_grad=True)
optimizer = torch.optim.Adam([thetas], lr=LEARNING_RATE)

def forward_kinematics(t):
    # Joint 1 position
    x1 = ORIGIN[0] + LINK_L1 * torch.cos(t[0])
    y1 = ORIGIN[1] + LINK_L1 * torch.sin(t[0])
    joint1 = torch.stack([x1, y1])
    
    # Joint 2 (End Effector) position
    x2 = x1 + LINK_L2 * torch.cos(t[0] + t[1])
    y2 = y1 + LINK_L2 * torch.sin(t[0] + t[1])
    eef = torch.stack([x2, y2])
    
    return joint1, eef

# --- Pygame Setup ---
pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
clock = pygame.time.Clock()
target_pos = torch.tensor([WIDTH // 2 + 100, HEIGHT // 2 - 100], dtype=torch.float32)

running = True
while running:
    screen.fill((30, 30, 30))
    
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.MOUSEBUTTONDOWN:
            mouse_x, mouse_y = pygame.mouse.get_pos()
            target_pos = torch.tensor([mouse_x, mouse_y], dtype=torch.float32)

    # --- IK Optimization Step ---
    optimizer.zero_grad()
    joint1_pos, eef_pos = forward_kinematics(thetas)
    
    # Loss is the distance between End Effector and Target
    loss = torch.nn.functional.mse_loss(eef_pos, target_pos)
    
    if loss > 0.1: # Threshold to stop optimizing when close enough
        loss.backward()
        optimizer.step()

    # --- Drawing ---
    # Draw Target
    pygame.draw.circle(screen, (255, 0, 0), target_pos.detach().numpy().astype(int), 8)
    
    # Convert tensors to numpy for drawing
    p0 = ORIGIN.numpy().astype(int)
    p1 = joint1_pos.detach().numpy().astype(int)
    p2 = eef_pos.detach().numpy().astype(int)
    
    # Draw Links
    pygame.draw.line(screen, (200, 200, 200), p0, p1, 5)
    pygame.draw.line(screen, (200, 200, 200), p1, p2, 5)
    
    # Draw Joints
    pygame.draw.circle(screen, (0, 255, 255), p0, 6)
    pygame.draw.circle(screen, (0, 255, 255), p1, 6)
    pygame.draw.circle(screen, (255, 255, 0), p2, 6)

    pygame.display.flip()
    clock.tick(60)

pygame.quit()
