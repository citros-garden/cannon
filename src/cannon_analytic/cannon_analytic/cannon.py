import math

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


	def analytic_step(self):
		self.vel[0] = self.vel_orig[0] + self.acc[0] * self.time
		self.vel[1] = self.vel_orig[1] + self.acc[1] * self.time

		self.pos[0] = self.pos_orig[0] + (self.vel_orig[0] + (0.5) * self.acc[0] * self.time) * self.time
		self.pos[1] = self.pos_orig[1] + (self.vel_orig[1] + (0.5) * self.acc[1] * self.time) * self.time
		
		if (self.pos[1] < 0.0):
			self.impactTime = (-self.vel_orig[1] - math.sqrt(self.vel_orig[1] * self.vel_orig[1] - 2 * self.pos_orig[1]))/self.acc[1]
			self.pos[0] = self.impactTime * self.vel_orig[0]
			self.pos[1] = 0.0
			self.vel[0] = 0.0
			self.vel[1] = 0.0

			if not self.impact:
				self.impact = True
				self.logger.info(f"\n\nIMPACT: t = {self.impactTime}, pos[0] = {self.pos[0]}\n\n")
        
		# Increment time by the time delta that matches the frequency 
		# specified in the scheduler node.
		self.time += self.dt