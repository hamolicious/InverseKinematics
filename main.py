from math import degrees, radians
from random import randint
import pygame
from time import time
from pygame import constants

from vector.color import Color
from vector.vector2d import Vec2d
from vector.vector3d import Vec3d
from ik import InverseKinematicsActor2D, Constraints, Simulation


#region pygame init
pygame.init()
size = (600, 600)
screen = pygame.display.set_mode(size)
screen.fill([255, 255, 255])
pygame.display.set_icon(screen)
clock, fps = pygame.time.Clock(), 0

delta_time = 0 ; frame_start_time = 0
#endregion

ik = InverseKinematicsActor2D([300, 300], [50 for i in range(3)])
Simulation.simulation_step = 0.0001
Simulation.epsilon = 5
ik.bones[-1].constraint = lambda x : Constraints.clamp(0, 180, x)

while True:
	for event in pygame.event.get():
		if event.type == pygame.QUIT:
			pygame.quit()
			quit()
	frame_start_time = time()
	screen.fill(Color(150, 150, 255).get())

	mouse_pos   = Vec2d(pygame.mouse.get_pos())
	mouse_press = pygame.mouse.get_pressed()
	key_press   = pygame.key.get_pressed()

	if mouse_press[0]:
		ik.set_target_pos(mouse_pos)

	ik.update()
	ik.display(screen)

	pygame.display.update()
	clock.tick(fps)
	delta_time = time() - frame_start_time
	pygame.display.set_caption(f'Framerate: {int(clock.get_fps())}')