from math import degrees, radians
from random import randint
import pygame
from time import time
from vector.color import Color

from vector.vector2d import Vec2d
from vector.vector3d import Vec3d
from ik import InverseKinematicsActor

#region pygame init
pygame.init()
size = (600, 600)
screen = pygame.display.set_mode(size)
screen.fill([255, 255, 255])
pygame.display.set_icon(screen)
clock, fps = pygame.time.Clock(), 0

delta_time = 0 ; frame_start_time = 0
#endregion

class Spider:
	def __init__(self) -> None:
		self.pos = Vec2d(300, 270)

		self.body_radius = 20

		self.right_arm = InverseKinematicsActor(300, 50, 3)
		self.left_arm = InverseKinematicsActor(300, 50, 3)

		self.__preset_targets()

	def __preset_targets(self):
		dist = 100

		p = self.pos.copy()
		p.x += dist
		p.y = 300
		self.right_arm.set_target_pos(p)

		p = self.pos.copy()
		p.x -= dist
		p.y = 300
		self.left_arm.set_target_pos(p)

	def __position_arm_bases(self):
		p = self.pos.copy()
		p.x += self.body_radius
		self.right_arm.set_base_pos(p)
		p.x -= self.body_radius*2
		self.left_arm.set_base_pos(p)

	def __set_first_joints_angles(self):
		for i in range(1, 3):
			angle = self.left_arm.get_pose_vector_angle(i)
			self.left_arm.set_pose_vector_angle(-abs(angle), i)

			angle = self.right_arm.get_pose_vector_angle(i)
			self.right_arm.set_pose_vector_angle(abs(angle), i)

		angle = self.right_arm.get_pose_vector_angle(0)
		if angle < -117 : angle = -117
		if angle > 20   : angle = 20
		angle = self.right_arm.set_pose_vector_angle(angle, 0)

		angle = self.left_arm.get_pose_vector_angle(0)
		if angle > 243 : angle = 243
		if angle < 160 : angle = 160
		angle = self.left_arm.set_pose_vector_angle(angle, 0)

	def update(self):
		self.__position_arm_bases()

		self.left_arm.update()
		self.right_arm.update()

		self.__set_first_joints_angles()

	def display(self):
		pygame.draw.circle(screen, Color(255).get(), self.pos.get_int(), self.body_radius)

		self.left_arm.display(screen)
		self.right_arm.display(screen)

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

	if mouse_press[0] : spider.pos = mouse_pos.copy()
	spider.update()
	spider.display()

	pygame.draw.rect(screen, Color().get(), (0, 300, 600, 300))

	pygame.display.update()
	clock.tick(fps)
	delta_time = time() - frame_start_time
	pygame.display.set_caption(f'Framerate: {int(clock.get_fps())}')