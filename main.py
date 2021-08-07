import pygame
from time import time

from vector.color import Color
from vector.vector2d import Vec2d
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

Simulation.simulation_step = 0.0001
Simulation.epsilon = 1

class Spider:
	def __init__(self) -> None:
		self.body_rad = 25
		self.pos = Vec2d(300)

		self.legs = [
			InverseKinematicsActor2D(self.pos, [50, 50, 50]),
			InverseKinematicsActor2D(self.pos, [50, 50, 50])
		]

		self.__set_up_constraints()

		dist = Vec2d(100, 50)
		self.legs[0].set_target_pos(self.pos + dist)

		dist.x *= -1
		self.legs[1].set_target_pos(self.pos + dist)


	def __set_up_constraints(self):
		self.legs[0].bones[0].constraint = lambda x : Constraints.clamp(250, 350, x)
		self.legs[0].bones[1].constraint = lambda x : Constraints.clamp(0, 180, x)

		self.legs[1].bones[0].constraint = lambda x : Constraints.clamp(150, 300, x)
		self.legs[1].bones[1].constraint = lambda x : Constraints.clamp(250, 360, x)

	def __position_legs(self):
		self.legs[0].pos = self.pos + Vec2d(self.body_rad, 0)
		self.legs[1].pos = self.pos + Vec2d(-self.body_rad, 0)


	def update(self):
		self.__position_legs()

		print(self.legs[0].pose_vector[0])

		self.legs[0].update(30)
		self.legs[1].update(30)

	def display(self, screen):
		self.legs[0].display_debug(screen)
		self.legs[1].display_debug(screen)
		pygame.draw.circle(screen, Color(255).get(), self.pos.get(), self.body_rad)

spider = Spider()

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
		spider.pos.set(mouse_pos)

	spider.update()
	spider.display(screen)

	pygame.display.update()
	clock.tick(fps)
	delta_time = time() - frame_start_time
	pygame.display.set_caption(f'Framerate: {int(clock.get_fps())}')