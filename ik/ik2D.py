from math import radians
import pygame
from vector import Color, Vec2d, Vec3d
import numpy as np


class Simulation:
	simulation_step = 0.001
	epsilon = 10


class Constraints:
	def clamp(min_value: float, max_value: float, value: float):
		"""Clamps value to be between `min_value` and `max_value`

		Args:
			min_value (float): lower bracket
			max_value (float): upper bracket
			value (float): bone angle

		Returns:
			float: new bone angle
		"""
		value = min(value, max_value)
		value = max(value, min_value)
		return value

	def constant(constant_value: float, value: float):
		"""Sets the bone angle to a `constant_value`

		Args:
			constant_value (float): constant value to use
			value (float): bone angle

		Returns:
			float: new bone angle
		"""
		return constant_value


class Bone2D:
	def __init__(self, length: int, angle: float) -> None:
		self.length = length
		self.angle = angle

		self.constraint = lambda x : x

	def apply_constraint(self):
		"""Applies the callable `self.constraint`
		"""
		self.angle = self.constraint(self.angle) % 360


class InverseKinematicsActor2D:
	def __init__(self, pos: Vec2d, limb_lengths: list[int]) -> None:
		"""Solver and container for inverse kinematics

		Args:
			pos (Vec2d): base position of the 0th joint
			limb_lengths (list[int]): list of ints, the length of the list defines the amount of joints and the contents define the length of those limbs, i.e. limb_lengths=[100, 50, 12] will create a bone structure with 3 bones with lengths of 100, 50 and 12 respectively
		"""
		self.pos = Vec2d(pos)
		self.bones = self.__generate_bone_structure(limb_lengths)
		self.__target = self.__calculate_end_effector_pos().copy()


	#region Properties

	@property
	def pose_vector(self):
		return np.array([np.float64(bone.angle) for bone in self.bones])

	#endregion Properties


	def apply_new_pose_vector(self, pose_vector: list[float]) -> None:
		"""Applies the new pose vector to the bones

		Args:
			pose_vector (list[float] | np.ndarray): ndarray or list of floats that describe the positioning of the bone structure
		"""
		for bone, new_angle in zip(self.bones, pose_vector):
			bone.angle = new_angle

	def update(self, num_of_updates=25) -> None:
		"""Finds pose vector configurations to reach the target

		Args:
			num_of_updates (int, optional): num of times to calculate delta orientations. Defaults to 25.
		"""
		for _ in range(num_of_updates):
			self.__find_joint_configurations()

	def set_target_pos(self, pos: Vec2d):
		"""Reposition the target position

		Args:
			pos (Vec2d): new position for the target vector
		"""
		self.__target.set(pos)

	def display(self, screen: pygame.Surface, line_thickness=3):
		"""Displays the bone structure as lines to the `screen`

		Args:
			screen (pygame.Surface): pygame screen surface to draw to
			line_thickness (int, optional): thickness of lines. Defaults to 3.
		"""
		for p1, p2 in self.__calculate_points():
			pygame.draw.line(screen, Color(255).get(), p1.get_int(), p2.get_int(), line_thickness)

	def display_debug(self, screen: pygame.Surface):
		"""Displays the bone structure to the `screen` with additional information

		Args:
			screen (pygame.Surface): pygame screen surface to draw to
		"""
		for p1, p2 in self.__calculate_points():
			pygame.draw.line(screen, Color(255).get(), p1.get_int(), p2.get_int(), 2)
			pygame.draw.circle(screen, Color(255, 0, 0).get(), p1.get_int(), 2)

		pygame.draw.circle(screen, Color(0, 255, 0).get(), self.__target.get_int(), 2)


	def __generate_bone_structure(self, limb_lengths: list[int]) -> list[Bone2D]:
		return [Bone2D(length, 0) for length in limb_lengths]

	def __calculate_points(self) -> list[Vec2d]:
		current_pos = self.pos.copy()
		total_angle = 0
		results = []

		for bone in self.bones:
			bone.apply_constraint()

			total_angle += bone.angle

			next_pos = Vec2d.from_angle(radians(total_angle))
			next_pos.mult(bone.length)
			next_pos.add(current_pos)

			results.append((current_pos.copy(), next_pos.copy()))
			current_pos = next_pos.copy()

		return results

	def __calculate_end_effector_pos(self) -> Vec2d:
		return self.__calculate_points()[-1][-1]

	def __calculate_jacobian_transpose(self) -> np.ndarray:
		points = self.__calculate_points()
		end_effector_pos = self.__calculate_end_effector_pos()
		matrix = []

		for joint, _ in points:
			ja = np.cross(np.array([0, 0, 1]), (end_effector_pos - joint).get())
			matrix.append(ja)

		return np.array(matrix)

	def __calculate_delta_orientation(self) -> np.ndarray:
		jacobian_transpose = self.__calculate_jacobian_transpose()
		delta_location = (self.__target - self.__calculate_end_effector_pos()).get() + [0]
		delta_orientation = jacobian_transpose @ delta_location
		return delta_orientation

	def __find_joint_configurations(self) -> None:
		if self.__calculate_end_effector_pos().dist(self.__target) > Simulation.epsilon**2:
			delta_orientation = self.__calculate_delta_orientation()

			new_pose_vector = self.pose_vector
			new_pose_vector += delta_orientation * Simulation.simulation_step

			self.apply_new_pose_vector(new_pose_vector)







