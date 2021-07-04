
from math import radians
import pygame
from vector import Color, Vec2d, Vec3d
import numpy as np

SIMULATION_STEP = 0.001
EPSILON = 10

class InverseKinematicsActor:
	def __init__(self, pos, limb_length, num_of_limbs) -> None:
		self.pos = Vec2d(pos)
		self.__limb_length = limb_length
		self.__num_of_limbs = num_of_limbs

		self.__pose_vector = np.zeros(self.__num_of_limbs)
		self.__target = self.__calculate_end_effector_pos().copy()

	def set_pose_vector_angle(self, new_angle, index):
		self.__pose_vector[index] = new_angle

	def get_pose_vector_angle(self, index):
		return self.__pose_vector[index]

	def set_base_pos(self, pos):
		self.pos.set(pos)

	def set_target_pos(self, pos):
		self.__target.set(pos)

	def update(self, num_of_updates=25):
		for _ in range(num_of_updates):
			self.__find_joint_configurations()

	def display(self, screen, line_thickness=3):
		for p1, p2 in self.__calculate_points():
			pygame.draw.line(screen, Color(255).get(), p1.get_int(), p2.get_int(), line_thickness)

	def display_debug(self, screen):
		for p1, p2 in self.__calculate_points():
			pygame.draw.line(screen, Color(255).get(), p1.get_int(), p2.get_int(), 2)
			pygame.draw.circle(screen, Color(255, 0, 0).get(), p1.get_int(), 2)

		pygame.draw.circle(screen, Color(0, 255, 0).get(), self.__target.get_int(), 2)


	def __calculate_points(self):
		current_pos = self.pos.copy()
		total_angle = 0
		results = []

		for angle in self.__pose_vector:
			total_angle += angle

			next_pos = Vec2d.from_angle(radians(total_angle))
			next_pos.mult(self.__limb_length)
			next_pos.add(current_pos)

			results.append((current_pos.copy(), next_pos.copy()))
			current_pos = next_pos.copy()

		return results

	def __calculate_end_effector_pos(self):
		return self.__calculate_points()[-1][-1]

	def __calculate_jacobian_transpose(self):
		points = self.__calculate_points()
		end_effector_pos = self.__calculate_end_effector_pos()
		matrix = []

		for joint, _ in points:
			ja = np.cross(np.array([0, 0, 1]), (end_effector_pos - joint).get())
			matrix.append(ja)

		return np.array(matrix)

	def __calculate_delta_orientation(self):
		jacobian_transpose = self.__calculate_jacobian_transpose()
		delta_location = (self.__target - self.__calculate_end_effector_pos()).get() + [0]
		delta_orientation = jacobian_transpose @ delta_location
		return delta_orientation

	def __find_joint_configurations(self):
		if self.__calculate_end_effector_pos().dist(self.__target) > EPSILON:
			delta_orientation = self.__calculate_delta_orientation()
			self.__pose_vector += delta_orientation * SIMULATION_STEP
		else : return True


