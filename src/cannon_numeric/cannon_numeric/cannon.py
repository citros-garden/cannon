import math
import numpy as np

class cannon:

	def __init__(self, logger):
		self.logger = logger
		self.cannon_default_data()
		self.init_calculations()


	def cannon_default_data(self):
			self.pos_orig = [0.0, 0.0]
			self.pos = [0.0, 0.0]
			self.acc = [0.0, -9.81]
			self.time = 0.0
			self.impact = False
			self.impactTime = 0.0

			# ROS parameters
			self.init_speed = 50.0
			self.init_angle = 30.0
			self.dt = 0.01


	def init_calculations(self):
		self.vel_orig = [math.cos(math.radians(self.init_angle))*self.init_speed, 
						 math.sin(math.radians(self.init_angle))*self.init_speed]
		self.vel = [self.vel_orig[0], self.vel_orig[1]]


	def cannon_integ(self):
		# assume constant acceleration
		y0 = [self.acc[0], self.acc[0]]
		y1 = [self.acc[1], self.acc[1]]

		# integrate 
		new_vel0 = np.trapz(y0,dx=0.01) + self.vel[0]
		new_vel1 = np.trapz(y1,dx=0.01) + self.vel[1]
		new_pos0 = np.trapz([self.vel[0], new_vel0], dx=self.dt) + self.pos[0]
		new_pos1 = np.trapz([self.vel[1], new_vel1], dx=self.dt) + self.pos[1]

		# update state
		self.vel[0] = new_vel0
		self.vel[1] = new_vel1
		self.pos[0] = new_pos0
		self.pos[1] = new_pos1

		self.time += self.dt

		if (self.pos[1] < 0.0):
			self.impactTime = self.time
			self.pos[1] = 0.0
			self.vel[0] = 0.0
			self.vel[1] = 0.0
			if not self.impact:
				self.impact = True
				self.logger.info(f"\n\nIMPACT: t = {self.impactTime}, pos[0] = {self.pos[0]}\n\n")
